import test from "node:test";
import assert from "node:assert/strict";
import { initialTelemetry } from "../../packages/contracts/src/telemetry";
import { landingHeight } from "../../packages/sim-browser/src/session/landing-height";
import { landingMission } from "../../apps/simulator/missions/default";
import { validateMission } from "../../packages/contracts/src/mission";

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
test("landing example is a native approach followed by NAV_LAND", () => {
  const items = validateMission(landingMission());
  assert.deepEqual(
    items.map((w) => w.command),
    [16, 16, 21],
  );
  assert.equal(items[2].alt, 0);
  assert(items[0].lat < items[1].lat && items[1].lat < items[2].lat);
});
