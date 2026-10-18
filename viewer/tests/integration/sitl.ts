import { HOME, exampleMission } from "../../apps/simulator/missions/default";
/** Integration test against the packaged firmware, without browser mocks. */
import assert from "node:assert/strict";
import { pathToFileURL } from "node:url";
import { resolve } from "node:path";
import { readFile } from "node:fs/promises";
import { validate } from "@microagu/contracts/recordings";
import { Simulator } from "@microagu/sim-browser/session/runtime";

Object.defineProperty(globalThis, "crossOriginIsolated", { value: true });
const sim = new Simulator(HOME);
sim.onLog = (text) => console.log(text);
sim.onError = (text) => {
  console.error(text);
  console.error(
    "Physics at failure",
    sim.physics.launchElapsed,
    sim.physics.state,
    sim.physics.command,
  );
  process.exit(1);
};
let truthFrames = 0,
  maxJoint = 0;
const launchStages = new Set<string>();
let inverted = false,
  canopyLow = false,
  canopyHigh = false,
  takeoffSeen = false;
sim.onTruth = (frame) => {
  validate(frame);
  truthFrames++;
  takeoffSeen ||= sim.state.mode === 13;
  maxJoint = Math.max(maxJoint, Math.hypot(...frame.joint_rotvec_rad));
  if (frame.launch_stage) launchStages.add(frame.launch_stage);
  if (frame.launch_stage === "Inverted setup")
    inverted = frame.state[4] ** 2 > 0.8;
  if (frame.launch_stage === "Upright initialization")
    canopyLow = frame.state[15] > -0.2;
  if (frame.launch_stage === "Two steps") canopyHigh = frame.state[15] < -1.8;
  if (frame.launch_stage === "Free flight")
    assert.equal(frame.launch_phase, 2, "Launch must not touch down");
  if (frame.launch_stage && frame.launch_stage !== "Free flight")
    assert.equal(frame.rpm, 0, "Propeller stays stopped through handling");
};
const timeout = setTimeout(() => {
  console.error("Integration timeout");
  process.exit(1);
}, 180000);
try {
  await sim.start(
    pathToFileURL(resolve("generated/ardupilot/arduplane.js")).href,
    JSON.parse(await readFile("assets/runtime/highdof.json", "utf8")),
    pathToFileURL(resolve("generated/physics/")).href + "/",
  );
  console.log("BOOT OK");
  const normalStart = sim.physics.time;
  const wallStart = performance.now();
  await new Promise((r) => setTimeout(r, 1500));
  const rate =
    (sim.physics.time - normalStart) / ((performance.now() - wallStart) / 1000);
  assert.ok(
    rate > 0.7 && rate < 1.3,
    `Warm-up must return to real time: ${rate}`,
  );
  assert.ok(Math.abs(sim.state.alt) < 2, "EKF must initialize on the ground");
  for (const mode of ["MANUAL", "FBWA", "FBWB", "CRUISE"]) {
    await sim.mode(mode);
    console.log("MODE OK", mode);
  }
  await sim.upload(exampleMission(), (n) => console.log("MISSION REQUEST", n));
  console.log("UPLOAD OK");
  // Give GPS/EKF initialization a bounded period, retaining normal arming checks.
  let armed = false;
  for (let i = 0; i < 12; i++) {
    try {
      await sim.launch("AUTO");
      armed = true;
      break;
    } catch (e) {
      console.log("Waiting for prearm:", String(e));
      await new Promise((r) => setTimeout(r, 3000));
    }
  }
  assert.ok(
    armed && sim.physics.released,
    "Paraglider must arm and release airborne physics",
  );
  assert.ok(
    sim.state.alt >= 15 && sim.state.alt < 20,
    `Ground home datum must be preserved: ${sim.state.alt}`,
  );
  console.log("GROUND LAUNCH OK", JSON.stringify(sim.state));
  assert.ok(takeoffSeen, "ArduPlane TAKEOFF must own the climb");
  assert.ok(
    inverted && canopyLow && canopyHigh,
    "Inverted launch must move the real canopy truth from ground to overhead",
  );
  for (const stage of [
    "Inverted setup",
    "Overhead swing",
    "Two steps",
    "Light toss",
  ])
    assert.ok(launchStages.has(stage), stage);
  await sim.execute();
  assert.equal(sim.state.mode, 10);
  console.log("AUTO OK");
  sim.physics.setWind(1.5, 270, 0.2);
  const before = { ...sim.state };
  await new Promise((r) => setTimeout(r, 12000));
  assert.equal(
    sim.physics.launchPhase,
    2,
    "Assisted launch must release into free flight",
  );
  assert.ok(
    sim.state.alt > 3,
    `AUTO must climb after launch: ${sim.state.alt}`,
  );
  assert.ok(
    Math.abs(sim.state.lat - before.lat) +
      Math.abs(sim.state.lon - before.lon) >
      0.00005,
    "Aircraft must fly",
  );
  assert.ok(
    sim.state.mission >= 1,
    "Mission must advance to a navigation item",
  );
  console.log("FLIGHT OK", JSON.stringify(sim.state));
  for (const mode of ["AUTO", "LOITER"]) {
    await sim.mode(mode);
    for (const direction of [1, -1]) {
      sim.controls(direction, 0, 0.45);
      await new Promise((r) => setTimeout(r, 2000));
      assert.ok(
        direction * sim.state.targetRoll! > 1,
        `${mode} must mix pilot steering: ${sim.state.targetRoll}`,
      );
    }
    sim.controls(0, 0, 0.45);
    console.log("AUTOPILOT NUDGE OK", mode);
  }
  await sim.mode("FBWA");
  sim.controls(0.6, 0, 0.6);
  const yaw = sim.state.yaw;
  await new Promise((r) => setTimeout(r, 8000));
  const change = Math.atan2(
    Math.sin(sim.state.yaw - yaw),
    Math.cos(sim.state.yaw - yaw),
  );
  assert.ok(
    Math.abs(change) > 0.2,
    "Pilot RC input must change aircraft heading",
  );
  console.log("STEERING OK", change);
  sim.controls(0, 0, 0.45);
  await sim.mode("MANUAL");
  await new Promise((r) => setTimeout(r, 2000));
  assert.equal(sim.state.mode, 0);
  await sim.mode("CRUISE");
  assert.equal(sim.state.mode, 7);
  await sim.altitude(45);
  assert.ok(
    Math.abs(sim.state.targetAlt! - 45) < 1,
    "CRUISE must accept an actual AP altitude demand",
  );
  await new Promise((r) => setTimeout(r, 2500));
  assert.ok(
    Math.abs(sim.state.targetAlt! - 45) < 1,
    "CRUISE demand must persist after heading locks",
  );
  await assert.rejects(sim.altitude(NaN));
  console.log("MANUAL TAKEOVER AND CRUISE OK");
  for (const mode of ["FBWB", "CRUISE"]) {
    await sim.mode(mode);
    sim.controls(0, 0, 0.5);
    await sim.altitude(45);
    sim.controls(0, 0, 1);
    await new Promise((r) => setTimeout(r, 2500));
    assert.ok(
      sim.state.targetAlt! > 46,
      `${mode} lever must raise AP altitude target: ${sim.state.targetAlt}`,
    );
    sim.controls(0, 0, 0.5);
    await new Promise((r) => setTimeout(r, 1000));
    const hold = sim.state.targetAlt!;
    await new Promise((r) => setTimeout(r, 1000));
    assert.ok(
      Math.abs(sim.state.targetAlt! - hold) < 0.3,
      `${mode} release must hold target`,
    );
    sim.controls(0, 0, 0);
    await new Promise((r) => setTimeout(r, 2000));
    assert.ok(
      sim.state.targetAlt! < hold - 1,
      `${mode} lever must lower AP altitude target`,
    );
    sim.controls(0, 0, 0.5);
    console.log("ALTITUDE LEVER OK", mode);
  }
  assert.ok(
    truthFrames > 50 && maxJoint > 0.01,
    "Independent canopy/payload truth must stream",
  );
  console.log("9-DOF TRUTH OK", {
    truthFrames,
    maxJoint,
    simTime: sim.physics.time,
  });
  clearTimeout(timeout);
  process.exit(0);
} catch (e) {
  console.error(e);
  clearTimeout(timeout);
  process.exit(1);
}
