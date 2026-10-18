# Native firmware MIMO integration — 16 October 2026

AI-assisted implementation and tests. These runs use the packaged ArduPlane WASM,
its normal EKF3, and the external Rust `as_built` full-DOF model. They are simulation
integration evidence, not flight validation or a claim of LQI superiority.

Reusable synthesis belongs in `research/archive/legacy-control/design_mimo.py`, generated model
artifacts in `simulation/data/control/mimo`, and firmware code in ArduPilot's
`libraries/AP_CustomControl`. Only one-off comparison results live here.

## Paired diagnostic

Both runs load the default 60 m AUTO mission. Wind is 1 m/s from east, turbulence
0.2 m/s, and the default 1.5 m/s thermal is centred at N300/E0 with radius 60 m.
The controller is selected 20 simulated seconds after launch, then sampled for
100 seconds. Metrics use seconds 20–100 of that interval. Weather is deterministic
in simulated time; browser/firmware boot timing and estimator realizations are not
bit-identical. This is one scenario per controller, not a statistical study.

| Metric | PG + TECS | Revised LQI |
|---|---:|---:|
| Altitude error RMS | 4.73 m | 8.65 m |
| Cross-track error RMS | 3.89 m | 4.73 m |
| Payload pitch RMS | 0.1335 rad | 0.1367 rad |
| Final mission item | 3 | 3 |

PG + TECS remains the default. LQI completes the same initial waypoint sequence,
but does not outperform the deployed baseline here. Its local fixed model,
conservative climb/turn governor and observer need further development before
flight use. Altitude error is against the AP navigation demand, not the backend's
slower governed reference. Cross-track telemetry can be zero during the first
waypoint approach; it must not be interpreted independently of mission progress.

`rejected-initial-lqi.json` records the first implementation, which stayed airborne
but did not reach waypoint 1. It had no equilibrium turn feed-forward and froze all
integrals when any actuator saturated. The revised design adds model-derived
turn/climb/speed feed-forward and full-matrix integral back-calculation. That change
was followed by another full run; the failed candidate was not published.

The reports include firmware WASM hashes. The embedded source revision in these
development builds predates the uncommitted changes; use the binary hashes and
the final source commits, not that revision alone, to identify the tested code.

## Other verification

- Four native unit tests: coupled brake travel, slew/reduced authority, invalid
  vectors, and engagement/estimator-reset/stale-input behavior.
- Native Plane SITL: MIMO switch, existing independent-axis custom-control return,
  and landing/flare regression tests passed. These use the built-in model.
- Full Rust physics WASM smoke: 20 seconds continuous LQI, return to PG+TECS,
  MANUAL exclusion, CRUISE re-entry, an altitude command and explicit left/right
  steering checks; EKF remains in the loop.
- Packaged-firmware handover continuity: maximum observed throttle-command
  changes between truth frames were 0.8 percentage points entering LQI and 0.6
  points returning to baseline (0.4 points on the later CRUISE return). The test
  rejects a step above 8 percentage points during the switch window.
- Browser test: selection and active-state feedback, reset, touch layouts and
  existing plots/trail/weather controls. Viewer unit suite: 40 tests passed.

To repeat the diagnostic from `viewer`:

```sh
npx tsx tests/integration/control-comparison.ts baseline /tmp/baseline.json
npx tsx tests/integration/control-comparison.ts lqi /tmp/lqi.json
npm run test:mimo
```
