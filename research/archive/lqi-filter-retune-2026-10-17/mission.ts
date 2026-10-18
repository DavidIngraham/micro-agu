/** One process per paired-study case: real packaged AP, EKF and external physics. */
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
} = JSON.parse(await readFile(casePath, "utf8"));
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
let latestFrame: number[] | undefined;
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
    }) + "\n",
  );
}
const timeout = setTimeout(async () => {
  failure = "Wall-clock timeout";
  await save();
  process.exit(1);
}, 300000);
const delay = () => new Promise((r) => setTimeout(r, 10));
try {
  await sim.start(
    pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    config,
    pathToFileURL(resolve("generated/physics/")).href + "/",
  );
  await sim.upload(exampleMission(), () => {});
  await sim.launch("AUTO");
  await sim.speedup(spec.speedup);
  const launch = sim.physics.time;
  while (sim.physics.time < launch + 30 && !failure) await delay();
  if (spec.mode !== "AUTO") await sim.mode(spec.mode);
  await sim.controller(scheme as "baseline" | "lqi");
  start = sim.physics.time;
  while (sim.physics.time < start + 180 && !failure) {
    const t = sim.physics.time - start;
    if (t >= 20 && phase === "settle") {
      let from=spec.from, weather=spec.weather;
      if(spec.mode==='CRUISE') {
        if(!latestFrame)throw Error('Missing pose for matched CRUISE weather');
        const [qx,qy,qz,qw]=latestFrame.slice(3,7);
        const yaw=Math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
        from=(from+yaw*180/Math.PI+360)%360;
        weather={...weather,
          thermalNorth:latestFrame[0]+Math.cos(yaw)*weather.thermalNorth-Math.sin(yaw)*weather.thermalEast,
          thermalEast:latestFrame[1]+Math.sin(yaw)*weather.thermalNorth+Math.cos(yaw)*weather.thermalEast};
      }
      sim.physics.setWind(spec.wind, from, 0, weather);
      phase = "disturbed";
    }
    if (t >= 100 && phase === "disturbed") {
      sim.physics.setWind(0, 0, 0, calmWeather);
      phase = "recovery";
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
