import test from "node:test";
import assert from "node:assert/strict";
import { initialTelemetry } from "../../packages/contracts/src/telemetry";
import { landingHeight } from "../../packages/sim-browser/src/session/landing-height";
import { LandingGuidance } from "../../packages/sim-browser/src/session/landing-guidance";

test("EKF HAGL propagates telemetry age, without applying a second sensor delay", () => {
  const s = initialTelemetry({ lat: 0, lon: 0 });
  s.hagl = { metres: 1, valid: true, timeBootMs: 10000, receivedTimeS: 10.05 };
  s.navigationTimeS = 10.05;
  s.climb = -2;
  assert(Math.abs(landingHeight(s, 10.1)! - 0.8) < 1e-10);
  assert.equal(landingHeight(s, 10), 1);
  assert.equal(landingHeight(s, 10.4), null);
  s.hagl.valid = false;
  assert.equal(landingHeight(s, 10.1), null);
  s.hagl.valid = true;
  s.navigationTimeS = 9;
  assert.equal(landingHeight(s, 10.1), null);
  s.navigationTimeS = 10;
  s.hagl.timeBootMs = 11000;
  assert.equal(landingHeight(s, 10.1), null);
});
test("bounded flare cuts throttle and latches despite a brief height rebound", () => {
  const g = new LandingGuidance(
    { north: 100, east: 0, heading: 0, speed: 4 },
    { feedback: true, flareHeight: 1.05, commonMm: 54 },
  );
  const m = {
    north: 95,
    east: 0,
    agl: 2,
    speed: 4,
    sink: 1.4,
    yaw: 0,
    pitch: 0,
    pitchRate: 0,
  };
  const approach = g.update(m, 0.05, false, 0);
  assert(!approach.flare);
  assert(approach.throttle >= 0.255 && approach.throttle <= 0.5);
  assert.equal(g.update(m, 0.05, true, 1).throttle, 0);
  const flare = g.update({ ...m, agl: 1, sink: 10 }, 0.05, true, 2);
  assert(flare.flare);
  assert.equal(flare.common, 58);
  assert.equal(flare.throttle, 0);
  assert(g.update({ ...m, agl: 1.2 }, 0.05, true, 2.05).flare);
});
