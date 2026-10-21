import test from "node:test";
import assert from "node:assert/strict";
import { TruthBuffer } from "@microagu/playback/live-buffer";
import type { Frame } from "@microagu/contracts/recordings";
const frame = (t: number): Frame => ({
  schema: "microagu.state.v1",
  time_s: t,
  state: [
    t,
    0,
    -100,
    0,
    0,
    0,
    1,
    1,
    0,
    0,
    0,
    0,
    0,
    t,
    0,
    -103,
    0,
    0,
    0,
    1,
    1,
    0,
    0,
    0,
    0,
    0,
  ],
  rpm: 8000,
  voltage_v: 22.2,
  brake_mm: [0, 0],
  throttle: 0.45,
  released: true,
  joint_rotvec_rad: [0, 0, 0],
  joint_rate_rad_s: [0, 0, 0],
});
test("jittered truth delivery never rewinds aircraft or rotor time", () => {
  const buffer = new TruthBuffer();
  let packet = 0,
    previous = 0;
  for (let tick = 0; tick < 1200; tick++) {
    const wall = tick / 120;
    while (packet * 0.04 + [0, 0.025, 0.005, 0.055][packet % 4] <= wall) {
      buffer.push(frame(packet * 0.04), wall);
      packet++;
    }
    const rendered = buffer.advance(1 / 120);
    if (!rendered) continue;
    assert.ok(
      rendered.time_s >= previous,
      "arrival must not reset playout time",
    );
    assert.ok(rendered.time_s <= (packet - 1) * 0.04, "never extrapolate");
    assert.ok(Math.abs(rendered.state[0] - rendered.time_s) < 1e-10);
    previous = rendered.time_s;
  }
  assert.ok(previous > 9.5, "bounded playout latency");
  buffer.push(frame(1), 11); // Late/out-of-order packet is ignored.
  assert.ok(buffer.advance(1 / 120)!.time_s >= previous);
});

test("clearing warm-up history starts presentation at the next live frame", () => {
  const buffer = new TruthBuffer();
  buffer.push(frame(1), 0);
  buffer.push(frame(5), 1);
  buffer.advance(0.016);
  buffer.clear();
  assert.equal(buffer.advance(0.016), null);
  buffer.push(frame(20), 2);
  assert.equal(buffer.advance(0.016)!.time_s, 20);
});

for (const speed of [2, 4, 8]) {
  test(`truth playback keeps up with ${speed}× simulation speed`, () => {
    const buffer = new TruthBuffer();
    let previous = 0;
    for (let tick = 0; tick <= 1200; tick++) {
      const wall = tick / 120;
      if (tick % 5 === 0) buffer.push(frame(wall * speed), wall);
      const rendered = buffer.advance(1 / 120)!;
      assert.ok(rendered.time_s >= previous, "playback never rewinds");
      assert.ok(rendered.time_s <= wall * speed, "never extrapolate");
      previous = rendered.time_s;
    }
    assert.ok(
      10 * speed - previous < 0.5 * speed,
      "less than 0.5 s wall-clock lag",
    );
  });
}
