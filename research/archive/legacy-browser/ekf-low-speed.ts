/** Research build with AP_NAVEKF3_LOW_SPEED_DIAGNOSTICS=1 required. */
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { HOME, exampleMission } from "../../apps/simulator/missions/default";
import { Simulator } from "@microagu/sim-browser/session/runtime";

Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const config = JSON.parse(await readFile("assets/runtime/highdof.json", "utf8"));
const sim = new Simulator(HOME);
const diagnostics: string[] = [];
let failure: string | null = null;
sim.onLog = (s) => { if (s.startsWith("N3LD,")) diagnostics.push(s); };
sim.onError = (s) => { failure = s; };
const delay = () => new Promise((r) => setTimeout(r, 10));
const timeout = setTimeout(() => { console.error("Timeout"); process.exit(1); }, 180000);
try {
  await sim.start(pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    config, pathToFileURL(resolve("generated/physics/")).href + "/");
  await sim.speedup(8);
  await sim.mode("MANUAL");
  await sim.command(400, [1]);
  const armed = sim.physics.time;
  while (sim.physics.time < armed + 30 && !failure) await delay();
  assert.equal(failure, null);
  assert.equal(diagnostics.length, 0, "Arming while held on the ground must not activate wind fusion");
  console.log("PASS 30 seconds armed on the ground without sideslip fusion");
  await sim.speedup(1);
  await sim.upload(exampleMission(), () => {});
  await sim.launch("AUTO");
  const launched = sim.physics.time;
  while (sim.physics.time < launched + 15 && !failure) await delay();
  assert.equal(failure, null);
  assert.ok(diagnostics.some((s) => Number(s.split(",")[3]) === 1), "Expected an applied airborne sideslip update");
  console.log("PASS airborne sideslip fusion after release", diagnostics.length);
  clearTimeout(timeout);
  process.exit(0);
} catch (e) {
  console.error(e);
  clearTimeout(timeout);
  process.exit(1);
}
