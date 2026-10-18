import { test } from "node:test";
import assert from "node:assert/strict";
import { Atmosphere } from "@microagu/sim-browser/physics/atmosphere";
import { calmWeather } from "@microagu/contracts/weather";

test("explicit gust seeds reproduce sequences and reset accumulated gust state", () => {
  const a = new Atmosphere(),
    b = new Atmosphere();
  const weather = { ...calmWeather, turbulence: 0.5, seed: 1234 };
  a.configure(1, 90, 0, weather);
  const first = a.sample(0.02, 0, 0);
  for (let i = 0; i < 20; i++) a.sample(0.02, 0, 0);
  a.configure(1, 90, 0, weather);
  b.configure(1, 90, 0, weather);
  assert.deepEqual(a.sample(0.02, 0, 0), first);
  assert.deepEqual(b.sample(0.02, 0, 0), first);
  b.configure(1, 90, 0, { ...weather, seed: 4321 });
  assert.notDeepEqual(b.sample(0.02, 0, 0), first);
  assert.throws(() => a.configure(0, 0, 0, { ...weather, seed: 0 }));
});

test("wind bearing, thermal centre and radius use NED consistently", () => {
  const air = new Atmosphere();
  assert.deepEqual(air.sample(0.005, 0, 0), [0, 0, 0]);
  air.configure(2, 0, 0.5, {
    ...calmWeather,
    thermalStrength: 3,
    thermalNorth: 100,
    thermalEast: -20,
    thermalRadius: 40,
  });
  assert.deepEqual(air.sample(0.005, 100, -20), [-2, 0, -3.5]);
  assert.ok(
    Math.abs(air.sample(0.005, 140, -20)[2] + 0.5 + 3 / Math.E) < 1e-12,
  );
  assert.ok(Math.abs(air.sample(0.005, 1000, 1000)[2] + 0.5) < 1e-12);
  assert.throws(() =>
    air.configure(0, 0, 0, { ...calmWeather, thermalRadius: 0 }),
  );
  assert.throws(() => air.configure(NaN, 0, 0));
});

test("gusts are repeatable, correlated, zero-mean and turn off completely", () => {
  const a = new Atmosphere(),
    b = new Atmosphere();
  for (const air of [a, b])
    air.configure(0, 0, 0, { ...calmWeather, turbulence: 1 });
  let sum = 0,
    square = 0,
    difference = 0,
    previous = 0;
  const count = 100000;
  for (let i = 0; i < count; i++) {
    const value = a.sample(0.02, 0, 0);
    assert.deepEqual(value, b.sample(0.02, 0, 0));
    sum += value[0];
    square += value[0] ** 2;
    difference += (value[0] - previous) ** 2;
    previous = value[0];
  }
  assert.ok(Math.abs(sum / count) < 0.15);
  assert.ok(square / count > 0.8 && square / count < 1.2);
  assert.ok(difference / count < 0.03);
  a.configure(0, 0, 0);
  assert.deepEqual(a.sample(0.02, 0, 0), [0, 0, 0]);
});
