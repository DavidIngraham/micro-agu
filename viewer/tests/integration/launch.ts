import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";
import { Physics } from "@microagu/sim-browser/physics/physics";
const config = JSON.parse(
  await readFile("assets/runtime/highdof.json", "utf8"),
);
const physics = await Physics.load(
  pathToFileURL(resolve("generated/physics")).href + "/",
  config,
);
physics.groundStart(true);
const stages = new Set<string>();
try {
  for (let i = 0; i < 4000; i++) {
    const packet = new Uint8Array(40),
      v = new DataView(packet.buffer);
    v.setUint16(0, 18458, true);
    v.setUint16(2, 200, true);
    v.setUint32(4, i, true);
    [1650, 1500, 800, 2400, ...Array(12).fill(1500)].forEach((p, c) =>
      v.setUint16(8 + c * 2, p, true),
    );
    physics.released = i >= 100;
    physics.respond(packet);
    stages.add(physics.launchStage!);
    if (!physics.powerReady) assert.equal(physics.rpm, 0);
    assert.ok(physics.inflation >= 0 && physics.inflation <= 1);
    if (physics.launchElapsed > 5.8)
      assert.equal(
        physics.launchPhase,
        2,
        "Glide and climb must stay clear of the ground",
      );
    if (i % 200 === 0)
      console.log(
        i,
        physics.launchStage,
        physics.state[2],
        physics.state[7],
        physics.rpm,
      );
  }
  assert.equal(physics.launchPhase, 2);
  for (const stage of [
    "Upright initialization",
    "Inverted setup",
    "Overhead swing",
    "Two steps",
    "Light toss",
    "Free flight",
  ])
    assert.ok(stages.has(stage), stage);
  console.log("PASS inverted launch and dynamic handoff");
} catch (e) {
  console.error(physics.launchElapsed, physics.state, physics.command);
  throw e;
} finally {
  physics.dispose();
}
