import { test } from "node:test";
import assert from "node:assert/strict";
import { equivalentAirspeed } from "@microagu/sim-browser/physics/airspeed";

test("JSON equivalent airspeed follows ArduPilot atmosphere reference densities", () => {
  // Reference values from AP_Baro/tests/test_baro_atmosphere.cpp.
  for (const [altitude, density] of [
    [-4000, 1.7697266],
    [6000, 0.66011156],
  ]) {
    const eas = equivalentAirspeed(10, altitude);
    assert.ok(Math.abs(eas - 10 * Math.sqrt(density / 1.225)) < 1e-5);
  }
  assert.ok(Math.abs(equivalentAirspeed(10, 0) - 10) < 1e-4);
  assert.ok(equivalentAirspeed(10, 650) < 10);
  assert.equal(equivalentAirspeed(0, 650), 0);
  assert.throws(() => equivalentAirspeed(NaN, 0));
  assert.throws(() => equivalentAirspeed(4, 12000));
});
