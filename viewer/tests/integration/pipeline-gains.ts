/** Verify the browser requires firmware acknowledgement of a compatible gain file. */
import assert from "node:assert/strict";
import { readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { Simulator } from "@microagu/sim-browser/session/runtime";
import { HOME } from "../../apps/simulator/missions/default";
Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const [firmware, gainPath, output, mode = "corrupt"] = process.argv.slice(2);
if (!firmware || !gainPath || !output || !["valid", "corrupt"].includes(mode))
  throw Error(
    "Usage: pipeline-gains.ts firmware.js gains.bin output.json valid|corrupt",
  );
const gains = new Uint8Array(await readFile(gainPath));
if (mode === "corrupt") gains[gains.length - 1] ^= 1;
const logs: string[] = [];
const sim = new Simulator(HOME);
sim.onLog = (s) => logs.push(s);
const timer = setTimeout(() => {
  console.error("Gain handshake test timed out");
  process.exit(1);
}, 150000);
try {
  const config = JSON.parse(
    await readFile("assets/runtime/highdof.json", "utf8"),
  );
  const start = sim.start(
    pathToFileURL(resolve(firmware)).href,
    config,
    pathToFileURL(resolve("generated/physics")).href + "/",
    gains,
  );
  if (mode === "corrupt") {
    await assert.rejects(start, /did not confirm a compatible LQI gain file/);
    assert(logs.some((s) => s.includes("LQI gain file rejected")));
    assert.equal(sim.state.controllerActive, false);
  } else {
    await start;
    assert(logs.some((s) => s.includes("LQI gain file loaded")));
  }
  await writeFile(
    output,
    JSON.stringify({ passed: true, mode, logs }, null, 2),
  );
  console.log(`Gain-file handshake passed: ${mode}`);
} finally {
  clearTimeout(timer);
  sim.dispose();
}
process.exit(0);
