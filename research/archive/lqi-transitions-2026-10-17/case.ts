/** Research harness copied temporarily into viewer/tests/integration by its runner. */
import { readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { HOME, exampleMission } from "../../apps/simulator/missions/default";
import { Simulator } from "@microagu/sim-browser/session/runtime";
import { calmWeather } from "@microagu/contracts/weather";
import type { Weather } from "@microagu/contracts/weather";
import type { PhysicsConfig } from "@microagu/sim-browser/physics/physics";

Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const [casePath, scheme, output] = process.argv.slice(2);
if (!casePath || !output || !["baseline", "lqi"].includes(scheme))
  throw Error("Usage: stability-case.ts case.json baseline|lqi output.json");
const spec: {
  id: number;
  mode: string;
  speedup: number;
  wind: number;
  from: number;
  weather: Weather;
  scales: Record<string, number>;
  brakeRateScale: number;
  parameters?: Record<string, number>;
  sensor?: string;
} = JSON.parse(await readFile(casePath, "utf8"));
const sensor = await import("../../../research/archive/lqi-transitions-2026-10-17/sensor.ts");
const config: PhysicsConfig = JSON.parse(
  await readFile("assets/runtime/highdof.json", "utf8"),
);
for (const [key, scale] of Object.entries(spec.scales)) {
  const index = config.parameter_keys.indexOf(key);
  if (index < 0) throw Error(`Unknown physics parameter ${key}`);
  config.parameters[index] *= scale;
}
config.brake_rate_mm_s *= spec.brakeRateScale;
for (const [name, value] of Object.entries(spec.parameters ?? {})) {
  if (!/^[A-Z][A-Z0-9_]{0,15}$/.test(name) || !Number.isFinite(value))
    throw Error(`Invalid research parameter ${name}`);
  config.defaults += `\n${name} ${value}\n`;
}
const sim = new Simulator(HOME);
sim.truthIntervalMs = 0;
const samples: unknown[] = [];
const logs: string[] = [];
const diagnostics: { time: number; values: number[] }[] = [];
const events: { time: number; name: string }[] = [];
let settleStart = 0;
let settledAfter = 0;
let latestFrame: number[] | undefined;
let requestedAltitude = 60;
let pitotEas = 0;
let tecsSpeedError = 0;
// Research-only read access to existing message listeners; no production API change.
(sim as unknown as {listeners: Set<(m: {name: string; values: Record<string, number>}) => void>}).listeners.add(m => {
  if (m.name === "VFR_HUD") pitotEas = Number(m.values.airspeed);
  if (m.name === "NAV_CONTROLLER_OUTPUT") tecsSpeedError = Number(m.values.aspd_error)/100;
});
let failure: string | null = null;
let start = Infinity,
  last = -Infinity,
  phase = "settle";
let done = false;
sim.onLog = (s) => {
  if (s.startsWith("LQID,")) {
    if (Number.isFinite(start))
      diagnostics.push({
        time: sim.physics.time - start,
        values: s.slice(5).split(",").map(Number),
      });
  } else logs.push(s);
};
sim.onError = (s) => {
  failure = s;
};
sim.onTruth = (frame) => {
  latestFrame = frame.state;
  const time = frame.time_s - start;
  if (time < 0 || time - last < 0.099) return;
  last = time;
  const s = sim.state;
  samples.push({
    time,
    phase,
    altitude: s.alt,
    targetAltitude: s.targetAlt,
    requestedAltitude,
    pitotEas,
    tecsSpeedError,
    crossTrack: s.crossTrack,
    roll: s.roll,
    pitch: s.pitch,
    yaw: s.yaw,
    targetRoll: s.targetRoll,
    active: s.controllerActive,
    mission: s.mission,
    truth: frame.state,
    rpm: frame.rpm,
    voltage: frame.voltage_v,
    command: [...sim.physics.command],
    brakes: [...sim.physics.brakes],
    wind: frame.wind_ned_m_s,
  });
  if (!frame.state.every(Number.isFinite)) failure = "Nonfinite physics";
  if (s.alt < 3) failure = "Altitude below 3 m";
  if (Math.abs(s.roll) > Math.PI / 3 || Math.abs(s.pitch) > Math.PI / 4)
    failure = "Attitude envelope exceeded (60 deg roll / 45 deg pitch)";
  if (time > 5 && s.controllerActive !== (scheme === "lqi"))
    failure = "Unexpected controller ownership";
};
async function save() {
  await writeFile(
    output,
    JSON.stringify({
      spec,
      scheme,
      complete: done,
      failure,
      build: JSON.parse(await readFile("assets/runtime/build.json", "utf8")),
      logs,
      diagnostics,
      samples,
      events,
      settledAfter,
    }) + "\n",
  );
}
const timeout = setTimeout(async () => {
  failure = "Wall-clock timeout";
  await save();
  process.exit(1);
}, 350000);
const delay = () => new Promise((r) => setTimeout(r, 10));
try {
  await sim.start(
    pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    config,
    pathToFileURL(resolve("generated/physics/")).href + "/",
  );
  sim.physics.researchSensor = sensor.makeSensor(spec.sensor ?? process.env.PROPRIO_CASE ?? "none", spec.id, sim.physics);
  await sim.upload(exampleMission(), () => {});
  await sim.launch("AUTO");
  await sim.speedup(spec.speedup);
  const launch = sim.physics.time;
  while (sim.physics.time < launch + 30 && !failure) await delay();
  if (spec.mode !== "AUTO") await sim.mode(spec.mode);
  await sim.controller(scheme as "baseline" | "lqi");
  await sim.altitude(60);
  const selected = sim.physics.time;
  settleStart = selected;
  let quietSince = selected;
  while (sim.physics.time-selected < 160 && !failure) {
    const y = latestFrame;
    const good = y && Math.abs(-y[2]-60)<.5 && Math.abs(y[9])<.1;
    if (!good) quietSince = sim.physics.time;
    if (sim.physics.time-quietSince >= 10) break;
    await delay();
  }
  if (sim.physics.time-selected >= 160) throw Error("Did not settle at 60 m before protocol");
  settledAfter = sim.physics.time-settleStart;
  start = sim.physics.time;
  const record = (name: string) => events.push({time:sim.physics.time-start,name});
  while (sim.physics.time < start + 320 && !failure) {
    const t = sim.physics.time - start;
    if (t >= 20 && phase === "settle") {
      phase = "climb";
      requestedAltitude = 70;
      record("climb-request");
      await sim.altitude(70);
      record("climb-ack");
    }
    if (t >= 120 && phase === "climb") {
      phase = "descend";
      requestedAltitude = 60;
      record("descent-request");
      await sim.altitude(60);
      record("descent-ack");
    }
    if (t >= 220 && phase === "descend") {
      phase = "vertical-gust";
      sim.physics.setWind(0, 0, 1, calmWeather);
      record("vertical-gust-on");
    }
    if (t >= 225 && phase === "vertical-gust") {
      phase = "vertical-recovery";
      sim.physics.setWind(0, 0, 0, calmWeather);
      record("vertical-gust-off");
    }
    if (t >= 260 && phase === "vertical-recovery") {
      phase = "crosswind-gust";
      sim.physics.setWind(2, 90, 0, calmWeather);
      record("crosswind-gust-on");
    }
    if (t >= 265 && phase === "crosswind-gust") {
      phase = "crosswind-recovery";
      sim.physics.setWind(0, 0, 0, calmWeather);
      record("crosswind-gust-off");
    }
    await delay();
  }
  done = !failure;
} catch (error) {
  failure = String(error);
}
clearTimeout(timeout);
await save();
console.log(
  JSON.stringify({
    id: spec.id,
    scheme,
    complete: done,
    failure,
    samples: samples.length,
  }),
);
process.exit(done ? 0 : 1);
