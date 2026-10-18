import { readFile } from "node:fs/promises";
import { resolve } from "node:path";
import { pathToFileURL } from "node:url";
import assert from "node:assert/strict";
import { Physics } from "@microagu/sim-browser/physics/physics";
import { equivalentAirspeed } from "@microagu/sim-browser/physics/airspeed";
import { validate } from "@microagu/contracts/recordings";
const config = JSON.parse(
  await readFile("assets/runtime/highdof.json", "utf8"),
);
const fixture = JSON.parse(
  await readFile("tests/fixtures/highdof-parity.json", "utf8"),
);
const physics = await Physics.load(
  pathToFileURL(resolve("generated/physics/")).href + "/",
  config,
);
let maxError = 0;
function compare(a: number[], b: number[], label: string, tolerance = 1e-7) {
  assert.equal(a.length, b.length, label);
  for (let i = 0; i < a.length; i++) {
    const error = Math.abs(a[i] - b[i]);
    maxError = Math.max(error, maxError);
    assert.ok(
      error < tolerance,
      `${label}[${i}]: ${a[i]} != ${b[i]} (${error})`,
    );
  }
}
for (let i = 0; i < fixture.steps; i++) {
  const pwm = [1500, 1500, 800, 2400, ...Array(12).fill(1500)];
  if (i >= 200 && i < 700) {
    pwm[0] = 1550;
    pwm[2] = 960;
    pwm[3] = 1900;
  }
  if (i >= 700) {
    pwm[0] = 1430;
    pwm[2] = 1450;
    pwm[3] = 2250;
  }
  const packet = new Uint8Array(40),
    view = new DataView(packet.buffer);
  view.setUint16(0, 18458, true);
  view.setUint16(2, 400, true);
  view.setUint32(4, i, true);
  pwm.forEach((v, c) => view.setUint16(8 + 2 * c, v, true));
  physics.released = i >= 20;
  const response = physics.respond(packet),
    sensors = JSON.parse(response);
  const expected = fixture.samples.find((s: any) => s.frame === i);
  if (!expected) continue;
  const truth = validate(physics.truth());
  compare(truth.state, expected.truth.state, "state");
  compare(truth.brake_mm!, expected.truth.brake_mm, "actuator");
  compare([truth.rpm!], [expected.truth.rpm], "RPM");
  compare(
    truth.joint_rotvec_rad,
    expected.truth.joint_rotvec_rad,
    "joint attitude",
  );
  compare(
    truth.joint_rate_rad_s,
    expected.truth.joint_rate_rad_s,
    "joint rate",
  );
  compare(
    sensors.imu.accel_body,
    expected.sensors.imu.accel_body,
    "IMU acceleration",
    1e-6,
  );
  compare(sensors.imu.gyro, expected.sensors.imu.gyro, "IMU gyro");
  compare(sensors.position, expected.sensors.position, "IMU position");
  compare(sensors.velocity, expected.sensors.velocity, "IMU velocity");
  // Duplicate packets must not advance time, actuator or rotor state.
  const time = physics.time;
  assert.equal(physics.respond(packet), response);
  assert.equal(physics.time, time);
}
console.log(
  `PASS: ${fixture.steps} steps, ${fixture.samples.length} native/Python parity checkpoints; maximum absolute difference ${maxError}`,
);
const wind = await Physics.load(
  pathToFileURL(resolve("generated/physics/")).href + "/",
  config,
  600,
);
const calm = await Physics.load(
  pathToFileURL(resolve("generated/physics/")).href + "/",
  config,
);
wind.setWind(2, 90, 0.5);
wind.released = calm.released = true;
let sensor: any;
for (let i = 0; i < 200; i++) {
  const packet = new Uint8Array(40),
    v = new DataView(packet.buffer);
  v.setUint16(0, 18458, true);
  v.setUint16(2, 200, true);
  v.setUint32(4, i, true);
  [1450, 1500, 800, 2400, ...Array(12).fill(1500)].forEach((p, c) =>
    v.setUint16(8 + c * 2, p, true),
  );
  sensor = JSON.parse(wind.respond(packet));
  calm.respond(packet);
}
compare(sensor.velocity_wind, [0, -2, -0.5], "wind NED");
compare(wind.truth().wind_ned_m_s!, [0, -2, -0.5], "wind truth");
assert.ok(
  Math.abs(wind.state[1] - calm.state[1]) > 0.01,
  "wind must affect actual physics",
);
assert.ok(
  Math.abs(
    sensor.airspeed -
      equivalentAirspeed(
        Math.hypot(wind.state[7], wind.state[8] + 2, wind.state[9] + 0.5),
        600 - sensor.position[2],
      ),
  ) < 1e-9,
);
assert.throws(() => wind.setWind(NaN, 0, 0));
assert.throws(() => wind.setWind(11, 0, 0));
wind.setWind(0, 0, 0);
assert.ok(wind.truth().wind_ned_m_s!.every((v) => v === 0));
console.log(
  "PASS: wind convention, sensor airspeed, physical response, reset and bounds",
);
wind.setWind(0, 0, 0, {
  turbulence: 0.3,
  thermalStrength: 1,
  thermalRadius: 40,
  thermalNorth: wind.state[13],
  thermalEast: wind.state[14],
});
let previousWind = wind.truth().wind_ned_m_s!;
let gustChanged = false;
for (let i = 200; i < 400; i++) {
  const packet = new Uint8Array(40),
    v = new DataView(packet.buffer);
  v.setUint16(0, 18458, true);
  v.setUint16(2, 200, true);
  v.setUint32(4, i, true);
  [1450, 1500, 800, 2400, ...Array(12).fill(1500)].forEach((p, c) =>
    v.setUint16(8 + c * 2, p, true),
  );
  const response = wind.respond(packet);
  const sensors = JSON.parse(response);
  const flow = wind.truth().wind_ned_m_s!;
  compare(sensors.velocity_wind, flow, "weather sensor/truth agreement");
  assert.ok(
    Math.abs(
      sensors.airspeed -
        equivalentAirspeed(
          Math.hypot(...wind.state.slice(7, 10).map((v, axis) => v - flow[axis])),
          600 - sensors.position[2],
        ),
    ) < 1e-9,
  );
  assert.equal(
    wind.respond(packet),
    response,
    "duplicate packets must not advance turbulence",
  );
  gustChanged ||= Math.abs(flow[0] - previousWind[0]) > 0.001;
  previousWind = flow;
}
assert.ok(gustChanged);
assert.ok(wind.truth().wind_ned_m_s![2] < -0.5, "thermal produces upward air");
console.log(
  "PASS: turbulence and thermal feed matching dynamics/sensor/truth wind",
);
const surfaceFixture = JSON.parse(
  await readFile("tests/fixtures/surface-parity.json", "utf8"),
);
const surface = await Physics.load(
  pathToFileURL(resolve("generated/physics/")).href + "/",
  config,
);
surface.groundStart();
for (let i = 0; i < surfaceFixture.steps; i++) {
  const packet = new Uint8Array(40),
    v = new DataView(packet.buffer);
  v.setUint16(0, 18458, true);
  v.setUint16(2, 200, true);
  v.setUint32(4, i, true);
  [i < 1600 ? 1650 : 1000, 1500, 800, 2400, ...Array(12).fill(1500)].forEach(
    (p, c) => v.setUint16(8 + 2 * c, p, true),
  );
  surface.released = i >= 20;
  const response = surface.respond(packet),
    sensors = JSON.parse(response);
  const expected = surfaceFixture.samples.find((s: any) => s.frame === i);
  if (!expected) continue;
  compare(
    surface.state,
    expected.truth.state,
    `surface state frame ${i}`,
    1e-7,
  );
  compare(
    sensors.imu.accel_body,
    expected.sensors.imu.accel_body,
    "surface acceleration",
    1e-6,
  );
  compare([surface.inflation], [expected.truth.inflation], "inflation");
  assert.equal(surface.launchPhase, expected.truth.launch_phase);
  assert.equal(surface.respond(packet), response);
}
assert.equal(surface.launchPhase, 3);
console.log(
  "PASS: 5000 native/WASM steps through ground hold, inflation, free flight and contact",
  { maxError },
);
