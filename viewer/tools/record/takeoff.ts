/** Capture the EKF/TAKEOFF launch used by the web simulator. Run from viewer. */
import assert from "node:assert/strict";
import { mkdir, readFile, writeFile } from "node:fs/promises";
import { createHash } from "node:crypto";
import { execFileSync } from "node:child_process";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { Simulator } from "@microagu/sim-browser/session/runtime";
import { validate, type Frame } from "@microagu/contracts/recordings";
import { HOME, exampleMission } from "../../apps/simulator/missions/default";
import { defaultWeather } from "../../apps/simulator/weather";
Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const sim = new Simulator(HOME),
  frames: Frame[] = [];
const weather = { ...defaultWeather, seed: 73 };
let start: number | undefined;
sim.truthIntervalMs = 0;
sim.onLog = console.log;
sim.onError = (message) => {
  throw Error(message);
};
sim.onTruth = (frame) => {
  if (start === undefined) return;
  const time = frame.time_s - start;
  if (frames.length && time - frames.at(-1)!.time_s < 0.05 - 1e-6) return;
  const mode = ({ 0: "MANUAL", 13: "TAKEOFF", 10: "AUTO" } as const)[
    sim.state.mode as 0 | 13 | 10
  ];
  assert.ok(mode, "Unexpected mode " + sim.state.mode);
  frames.push(
    validate({ ...frame, time_s: time, mode, mission_item: sim.state.mission }),
  );
};
const timeout = setTimeout(() => process.exit(1), 180000);
try {
  await sim.start(
    pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    JSON.parse(await readFile("assets/runtime/highdof.json", "utf8")),
    pathToFileURL(resolve("generated/physics")).href + "/",
  );
  await sim.mode("MANUAL");
  sim.physics.setWind(0, 0, 0, weather);
  await sim.upload(exampleMission(), () => {});
  start = sim.physics.time;
  await sim.launch("AUTO");
  await new Promise((r) => setTimeout(r, 8000));
  assert.ok(frames.some((f) => f.mode === "TAKEOFF"));
  assert.equal(frames.at(-1)!.mode, "AUTO");
  const out = "assets/recordings/takeoff";
  await mkdir(out, { recursive: true });
  const origin = frames[0].time_s;
  for (const frame of frames)
    frame.time_s = Number((frame.time_s - origin).toFixed(6));
  const data = frames.map((f) => JSON.stringify(f)).join("\n") + "\n";
  await writeFile(out + "/launch.jsonl", data);
  const hashes: Record<string, string> = {};
  for (const file of [
    "generated/ardupilot/arduplane.wasm",
    "generated/physics/sim_wasm_bg.wasm",
    "assets/runtime/highdof.json",
    "packages/sim-browser/src/session/runtime.ts",
  ])
    hashes[file] = createHash("sha256")
      .update(await readFile(file))
      .digest("hex");
  await writeFile(
    out + "/provenance.json",
    JSON.stringify(
      {
        source:
          "ArduPlane WASM + Rust 9-DOF, EKF enabled; recorded simulation, not flight-test data",
        revision: execFileSync("git", ["rev-parse", "HEAD"], {
          encoding: "utf8",
        }).trim(),
        command: "npx tsx tools/record/takeoff.ts",
        home: HOME,
        weather,
        recording_hz: 20,
        frames: frames.length,
        duration_s: frames.at(-1)!.time_s,
        assisted_release_s: 5.8,
        auto_handoff_altitude_m: 15,
        limitations:
          "Prescribed hand motion and scalar inflation until release; no cloth dynamics.",
        sha256: {
          ...hashes,
          "launch.jsonl": createHash("sha256").update(data).digest("hex"),
        },
      },
      null,
      2,
    ) + "\n",
  );
  console.log("Recorded " + frames.length + " frames");
} finally {
  clearTimeout(timeout);
  sim.onError = () => {};
  sim.dispose();
}
