# EKF-guided browser landing

The simulator now offers **LAND · ahead** from the main mode picker. It selects
the existing LQI controller for a straight approach, hands over at 3 m AGL to the
bounded brake flare, and pauses after estimated touchdown. Selecting another
mode cancels guidance. The target is computed ahead of the aircraft; this is not
return-to-launch navigation or a native ArduPlane LAND mode.

## Estimation and ownership

The generic lidar feeds the existing JSON `rng_1` → SITL rangefinder → EKF3
terrain-estimator path. ArduPlane publishes `AP_AHRS::get_hagl()` at 10 Hz as
`EKF_HAGL` named-float telemetry, with a negative invalid sentinel. This is current
output height relative to estimated terrain, not raw slant range. The browser
uses fused navigation velocity and attitude. It propagates only the short age
of that output to the worker's simulation clock. Both height and navigation
must be fresh within 0.3 s. Sensor capture delay is not compensated a second time.

Above 10 m, home-relative EKF altitude is allowed on the flat simulated field
when terrain height is unavailable. At lower heights, invalid/stale terrain
height aborts guidance. Before final, that requests a climb in CRUISE; after
handoff, power remains off and control is released to the user. No truth AGL,
relative canopy angles or airspeed sensor enters the controller.

The approach uses unchanged file-loaded LQI gains. The final uses the existing
54 mm common-brake feedforward with bounded sink feedback, 58 mm common limit,
and bounded differential steering through AP's MANUAL mixer. This is not LQI
controlling the flare. The shared implementation is under
`viewer/packages/sim-browser/src/session`, not imported from research code.

The physical sensor remains 20 Hz, 50 ms latency, 2 cm noise and 1% random
no-return probability. EKF3's range path assumes 25 ms delay. EKF HAGL therefore
does not remove all latency mismatch or terrain bias. The experiment keeps that
mismatch rather than selecting a perfect sensor.

## Landing endpoint

After flare, AGL below 0.3 m, vertical speed below 0.3 m/s, and ground speed below
2 m/s sustained for 0.25 s end the browser session. All are estimated quantities.
No force-disarm is sent: ArduPlane's airborne detector may still reject disarm
immediately after touchdown. The display says **PAUSED**, not DISARMED.

Preliminary traces showed a rigid-canopy ground-dynamics divergence about
3.1 s after a headwind landing, after a normal incoming sink of 0.77 m/s. Long
post-contact rollout/canopy collapse is outside this model's credible domain.
The endpoint avoids displaying that unsupported rollout. Ground friction and
attitude estimation during impact have not been flight-validated. Truth contact
is used only to measure incoming performance and verify that the estimated
endpoint did not precede physical touchdown.

## Reproduction

From the repository root, using the documented Node and Python environments:

```sh
control/.venv/bin/python research/active/landing-ekf-2026-10-20/run.py
control/.venv/bin/python research/active/landing-ekf-2026-10-20/assess.py
control/.venv/bin/python research/active/landing-ekf-2026-10-20/pack.py
control/.venv/bin/python research/active/landing-ekf-2026-10-20/validate.py
cd viewer
npm test
npm run build
npx playwright test tests/browser/landing.spec.ts
```

`protocol.json` pins the executed code, firmware, physics and gains.
`evidence.tar.gz` retains full flight traces and telemetry; `summary.json` and
`results.md` report pre-contact outcomes. Repeated development runs are not
statistical evidence. These are deterministic scenario checks, not a Monte
Carlo campaign or authorization for flight testing.
