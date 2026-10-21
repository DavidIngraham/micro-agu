import { missionPreview } from "../../apps/simulator/missions/preview";
import test from "node:test";
import assert from "node:assert/strict";
import { initialTelemetry } from "../../packages/contracts/src/telemetry";
import { landingHeight } from "../../packages/sim-browser/src/session/landing-height";
import {
  exampleMission,
  landingMission,
  HOME,
} from "../../apps/simulator/missions/default";
import { validateMission, offset } from "../../packages/contracts/src/mission";

test("default circuit repeats before the RTL landing sequence", () => {
  const items = validateMission(exampleMission());
  assert.deepEqual(
    items.map((w) => w.command),
    [16, 16, 16, 16, 16, 177, 189, 18, 31, 16, 21],
  );
  assert.deepEqual(items[5].params, [1, -1, 0, 0]);
  assert.deepEqual(items.slice(7), landingMission());
  const path = missionPreview(items, HOME);
  assert.equal(path.length, 137);
  assert(path.every((p) => Math.abs(p.north) < 500 && Math.abs(p.east) < 250));
  for (const params of [
    [0, -1, 0, 0],
    [12, -1, 0, 0],
    [1.5, -1, 0, 0],
    [1, -2, 0, 0],
  ]) {
    const invalid = structuredClone(items);
    invalid[5].params = params;
    assert.throws(() => validateMission(invalid), /invalid jump/);
  }
});

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
test("landing spiral exits tangent to the final approach", () => {
  const items = validateMission(landingMission());
  assert.deepEqual(
    items.map((w) => w.command),
    [18, 31, 16, 21],
  );
  const [capture, spiral, approach, land] = items;
  assert.equal(capture.params[0], 1);
  assert.equal(capture.alt, 60);
  assert.equal(spiral.alt, 30);
  assert.equal(spiral.params[1], 40); // positive = clockwise
  assert.equal(spiral.params[3], 1); // cross-track from actual tangent exit
  assert.equal(capture.lat, spiral.lat);
  assert.equal(capture.lon, spiral.lon);
  const c = offset(spiral.lat, spiral.lon, HOME);
  const a = offset(approach.lat, approach.lon, HOME);
  const l = offset(land.lat, land.lon, HOME);
  const dn = l.north - a.north,
    de = l.east - a.east;
  const distanceToLine =
    Math.abs(dn * (c.east - a.east) - de * (c.north - a.north)) /
    Math.hypot(dn, de);
  assert(Math.abs(distanceToLine - 40) < 1e-6);
  assert(c.north < a.north && a.north < l.north);
  assert.equal(approach.alt, spiral.alt);
  assert.equal(land.alt, 0);
});

test("preview shows descending orbit and a tangent exit rather than a centre-to-final diagonal", () => {
  const path = missionPreview(landingMission(), HOME);
  const exit = path[129],
    final = path[130];
  assert(Math.abs(exit.north + 280) < 1e-6);
  assert(Math.abs(exit.east) < 1e-6);
  assert.equal(exit.alt, 30);
  assert.equal(path[65].alt, 60);
  assert(Math.abs(final.east - exit.east) < 1e-6);
  assert(final.north > exit.north);
});
