/** Seeded launch regression using the packaged firmware, EKF and Rust model. */
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import { HOME, exampleMission } from "../../apps/simulator/missions/default";
import { defaultWeather } from "../../apps/simulator/weather";
import { Simulator } from "@microagu/sim-browser/session/runtime";

Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const timeout = setTimeout(() => process.exit(1), 180000);
const results = [];
try {
  for (const seed of [17, 73, 2026]) {
    const sim = new Simulator(HOME);
    const modes = new Set<number>();
    let minHeight = Infinity;
    let triggered = false;
    sim.onLog = (text) => {
      triggered ||= text.includes("Triggered AUTO");
    };
    sim.onError = (text) => {
      throw Error(text);
    };
    sim.truthIntervalMs = 0;
    sim.onTruth = (frame) => {
      if (!sim.physics.released) return;
      modes.add(sim.state.mode);
      if (!sim.physics.powerReady) assert.equal(frame.rpm, 0);
      if (sim.physics.launchElapsed > 5.8) {
        minHeight = Math.min(minHeight, -frame.state[2]);
        assert.equal(frame.launch_phase, 2, "No ground contact after release");
      }
    };
    try {
      await sim.start(
        pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
        JSON.parse(await readFile("assets/runtime/highdof.json", "utf8")),
        pathToFileURL(resolve("generated/physics")).href + "/",
      );
      sim.physics.setWind(0, 0, 0, { ...defaultWeather, seed });
      await sim.upload(exampleMission(), () => {});
      await sim.speedup(4);
      await sim.launch("AUTO");
      assert.ok(triggered, "AP must detect the launch");
      assert.ok(modes.has(13), "TAKEOFF must control the climb");
      assert.ok(!modes.has(5), "No FBWA workaround");
      assert.equal(sim.state.mode, 10);
      assert.ok(sim.state.alt >= 15);
      assert.ok(minHeight > 0.1);
      results.push({ seed, minHeight, handoffAltitude: sim.state.alt });
    } finally {
      sim.onError = () => {};
      sim.dispose();
    }
  }
  console.log("PASS seeded TAKEOFF launches", JSON.stringify(results));
} finally {
  clearTimeout(timeout);
}
