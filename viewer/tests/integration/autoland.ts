/** Native firmware integration: configurable speed, rejected entry and AUTOLAND. */
import assert from "node:assert/strict";
import { readFile, writeFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import definitions from "../../generated/mavlink/dialect.json";
import { HOME } from "../../apps/simulator/missions/default";
import { MODES } from "@microagu/contracts/telemetry";
import { calmWeather } from "@microagu/contracts/weather";

Object.assign(definitions, {
  PARAM_REQUEST_READ: {
    id: 20,
    crc: 214,
    length: 20,
    fields: [
      ["param_index", "int16_t", 0, 1],
      ["target_system", "uint8_t", 2, 1],
      ["target_component", "uint8_t", 3, 1],
      ["param_id", "char", 4, 16],
    ],
  },
});
const { Simulator } = await import("@microagu/sim-browser/session/runtime");
Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const sim = new Simulator(HOME);
const parameters = new Map<string, number>();
const logs: { time: number; speed: number; text: string }[] = [];
const phases = new Set<string>();
const modes = new Set<number>();
let failure: string | undefined,
  captureEnabled = false,
  landingStarted = false;
let activeApproach = false,
  inactiveFinal = false,
  maxFlareBrake = 0;
let contact: { miss: number; sink: number; groundSpeed: number } | undefined;
let previous: number[] | undefined;
const configBytes = await readFile("assets/runtime/highdof.json");
const firmwareBytes = await readFile("generated/ardupilot/arduplane.wasm");
const hash = (bytes: Uint8Array) =>
  createHash("sha256").update(bytes).digest("hex");
sim.onLog = (text) => {
  logs.push({ time: sim.physics?.time ?? 0, speed: sim.state.speed, text });
  console.log(text);
};
sim.onError = (text) => {
  failure = text;
};
(sim as any).listeners.add((m: any) => {
  if (m.name === "PARAM_VALUE")
    parameters.set(String(m.values.param_id), Number(m.values.param_value));
});
sim.truthIntervalMs = 0;
sim.onTruth = (frame) => {
  if (!landingStarted) return;
  modes.add(sim.state.mode);
  const phase = sim.state.landing?.phase;
  if (phase) phases.add(phase);
  if (phase === "approach") activeApproach ||= sim.state.controllerActive;
  if (phase === "final" || phase === "flare")
    inactiveFinal ||= !sim.state.controllerActive;
  if (phase === "flare")
    maxFlareBrake = Math.max(maxFlareBrake, ...sim.physics.brakes);
  if (!contact && (-frame.state[2] <= 0.12 || -frame.state[15] <= 0.05)) {
    const s = previous ?? frame.state;
    contact = {
      miss: Math.hypot(s[0], s[1]),
      sink: s[9],
      groundSpeed: Math.hypot(s[7], s[8]),
    };
  }
  previous = [...frame.state];
};
const delay = () => new Promise((r) => setTimeout(r, 20));
async function until(
  predicate: () => boolean,
  seconds: number,
  description: string,
) {
  const deadline = performance.now() + seconds * 1000;
  while (!predicate()) {
    if (failure) throw Error(failure);
    if (performance.now() > deadline) throw Error(`Timeout: ${description}`);
    await delay();
  }
}
async function readParam(name: string) {
  parameters.delete(name);
  sim.send("PARAM_REQUEST_READ", { param_id: name, param_index: -1 });
  await until(() => parameters.has(name), 5, `read ${name}`);
  return parameters.get(name)!;
}
async function setParam(name: string, value: number) {
  parameters.delete(name);
  sim.send("PARAM_SET", { param_id: name, param_value: value, param_type: 9 });
  await until(() => parameters.get(name) === value, 5, `set ${name}`);
}
const timeout = setTimeout(() => {
  console.error("AUTOLAND integration wall timeout");
  process.exit(1);
}, 240000);
try {
  await sim.start(
    pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    JSON.parse(configBytes.toString()),
    pathToFileURL(resolve("generated/physics/")).href + "/",
    new Uint8Array(await readFile("assets/runtime/lqi-gains.bin")),
  );
  assert.equal(await readParam("AHRS_MIN_SPEED"), 3);
  assert.equal(await readParam("AUTOLAND_WP_DIST"), 200);
  assert.equal(await readParam("AUTOLAND_WP_ALT"), 30);
  assert.equal(await readParam("LAND_FLARE_AIM"), 0);
  await assert.rejects(sim.mode("AUTOLAND"), /rejected command 176/);
  assert.notEqual(sim.state.mode, MODES.AUTOLAND);
  sim.physics.setWind(0, 0, 0, calmWeather);
  await sim.controller("lqi");
  await sim.speedup(8);
  await setParam("AHRS_MIN_SPEED", 20);
  await sim.launch("CRUISE");
  assert(!logs.some((l) => l.text.startsWith("Autoland direction=")));
  await assert.rejects(sim.mode("AUTOLAND"), /rejected command 176/);
  assert(
    logs.some((l) => l.text.includes("Takeoff initial direction not set")),
  );
  assert(sim.state.speed < 5, `Slow-aircraft test speed ${sim.state.speed}`);
  await setParam("AHRS_MIN_SPEED", 3);
  captureEnabled = true;
  await sim.mode("FBWA");
  await until(
    () => logs.some((l) => l.text.startsWith("Autoland direction=")),
    5,
    "low-speed heading capture",
  );
  const capture = logs.find((l) => l.text.startsWith("Autoland direction="))!;
  assert(
    capture.speed > 3 && capture.speed < 5,
    `Capture at ${capture.speed} m/s`,
  );
  await sim.mode("CRUISE");
  await sim.altitude(60);
  await until(() => sim.state.alt > 57, 35, "climb before landing");
  await sim.mode("AUTOLAND");
  await until(() => !!sim.state.landing, 5, "AUTOLAND presentation");
  assert.deepEqual(sim.state.landing!.target, { north: 0, east: 0 });
  await assert.rejects(sim.altitude(45), /Select another flight mode/);
  await sim.mode("CRUISE");
  await until(() => !sim.state.landing, 5, "abort clears landing state");
  assert(!sim.state.simulationEnded);
  await sim.mode("AUTOLAND");
  landingStarted = true;
  await until(
    () => !!sim.state.simulationEnded,
    140,
    "native landing and estimator-confirmed pause",
  );
  assert.equal(sim.state.mode, MODES.AUTOLAND);
  assert.equal(
    sim.state.mission,
    0,
    "AUTOLAND does not need an uploaded mission",
  );
  assert.equal(sim.state.landing?.phase, "landed");
  assert.deepEqual([...modes], [MODES.AUTOLAND]);
  for (const phase of ["approach", "final", "flare", "landed"])
    assert(phases.has(phase), phase);
  assert(
    activeApproach && inactiveFinal,
    "LQI must run on approach and hand over for flare",
  );
  assert(
    maxFlareBrake > 40,
    `Automatic symmetric brake flare ${maxFlareBrake}`,
  );
  assert(contact && contact.sink < 2.5, `Contact ${JSON.stringify(contact)}`);
  const result = {
    firmware_sha256: hash(firmwareBytes),
    configuration_sha256: hash(configBytes),
    checks: {
      parameter_readback: true,
      ground_entry_rejected: true,
      missing_direction_rejected: true,
      runtime_threshold_change: captureEnabled,
      capture_below_5_m_s: true,
      no_uploaded_mission: true,
      home_target: true,
      mode_exit_and_reentry: true,
      native_autoland_only: true,
      lqi_handover: true,
      automatic_brake_flare: true,
      estimator_confirmed_pause: true,
    },
    capture,
    phases: [...phases],
    contact,
    maxFlareBrake,
    final: sim.state,
    logs,
  };
  if (process.env.AUTOLAND_RESULT)
    await writeFile(
      process.env.AUTOLAND_RESULT,
      JSON.stringify(result, null, 2) + "\n",
    );
  console.log(
    "AUTOLAND PASS",
    JSON.stringify({ capture, contact, maxFlareBrake }),
  );
} finally {
  clearTimeout(timeout);
  sim.dispose();
}
