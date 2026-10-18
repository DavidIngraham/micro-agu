import assert from "node:assert/strict";
import test from "node:test";
import { Quaternion, Vector3 } from "../../../viewer/packages/contracts/src/math";
import { makeSensor, relativeChannels } from "./sensor";

test("relative frames reconstruct canopy-body rate independently of heading", () => {
  const p = new Quaternion(0, 0, Math.sin(Math.PI/4), Math.cos(Math.PI/4));
  const relative = new Quaternion(0, Math.sin(Math.PI/18), 0, Math.cos(Math.PI/18));
  const c = p.clone().multiply(relative);
  const y = Array(26).fill(0);
  y.splice(3, 4, ...p.toArray()); y.splice(16, 4, ...c.toArray());
  y.splice(10, 3, 0, 0, 1); y.splice(23, 3, 1, 0, 1);
  const measured = relativeChannels(y);
  measured.relative.toArray().forEach((v, i) => assert.ok(Math.abs(v-relative.toArray()[i]) < 1e-12));
  const body = new Vector3(0, 0, 1).applyQuaternion(p.clone().invert())
    .add(measured.rate).applyQuaternion(measured.relative.clone().invert());
  const expected = new Vector3(1, 0, 1).applyQuaternion(c.clone().invert());
  body.toArray().forEach((v, i) => assert.ok(Math.abs(v-expected.toArray()[i]) < 1e-12));
});

test("noisy angle sensor is seeded, delayed and does not output a truth-rate mode", () => {
  const a = makeSensor("sensor", 7), b = makeSensor("sensor", 7);
  const state = Array(26).fill(0); state[6] = state[19] = 1;
  const initial = a(state, 0); assert.deepEqual(initial, b(state, 0));
  state[16] = Math.sin(.1); state[19] = Math.cos(.1);
  assert.deepEqual(a(state, .02), initial); b(state, .02);
  assert.deepEqual(a(state, .04), initial); b(state, .04);
  assert.deepEqual(a(state, .06), b(state, .06));
  assert.equal(initial.microagu_mode, 3);
});

test("ideal output uses scalar-first quaternion and disables explicitly", () => {
  const state = Array(26).fill(0); state[6] = state[19] = 1;
  assert.deepEqual(makeSensor("angles", 1)(state, 0).microagu_relative, [1, 0, 0, 0]);
  assert.equal(makeSensor("none", 1)(state, 0).microagu_mode, 0);
});
