import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { execFileSync } from "node:child_process";
import init, { Dryden } from "../../generated/physics/sim_wasm";
import { Atmosphere } from "@microagu/sim-browser/physics/atmosphere";
import { calmWeather } from "@microagu/contracts/weather";

await init({
  module_or_path: await readFile(
    new URL("../../generated/physics/sim_wasm_bg.wasm", import.meta.url),
  ),
});
const reference = JSON.parse(
  execFileSync(
    "../simulation/.venv/bin/python",
    [
      "-c",
      "import json; from microagu_sim import Dryden; d=Dryden(73); print(json.dumps([d.step(.02,4.2,[20.,15.,10.],[.3,.2,.1]) for _ in range(100)]))",
    ],
    { encoding: "utf8" },
  ),
);
const d = new Dryden(73);
for (const expected of reference) {
  const actual = d.step(
    0.02,
    4.2,
    new Float64Array([20, 15, 10]),
    new Float64Array([0.3, 0.2, 0.1]),
  );
  actual.forEach((v, i) => assert.ok(Math.abs(v - expected[i]) < 1e-12));
}
d.free();
const air = new Atmosphere((seed) => new Dryden(seed));
const weather = {
  ...calmWeather,
  turbulenceModel: "dryden" as const,
  gustLength: 20,
  verticalRatio: 0.5,
};
air.configure(2, 0, 0.5, weather);
assert.deepEqual(air.sample(0, 0, 0), [0, 0, 0]);
let v: number[] = [];
for (let i = 0; i < 2000; i++) v = air.sample(0.01, 0, 0, [4, 0, 0]);
assert.ok(Math.abs(v[0] + 2) < 0.001 && Math.abs(v[2] + 0.5) < 0.001);
air.configure(2, 0, 0.5, { ...weather, turbulence: 0.3, verticalRatio: 0 });
for (let i = 0; i < 100; i++) {
  v = air.sample(0.01, 0, 0, [4, 0, 0]);
  assert.ok(Math.abs(v[2] + 0.5) < 0.001);
}
assert.throws(() => air.configure(0, 0, 0, { ...weather, gustLength: 0 }));
air.dispose();
console.log(
  "Dryden Python/WASM parity, NED direction, weather ramp and zero vertical gust passed",
);
