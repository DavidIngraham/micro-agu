# Native AUTO landing validation

[Results](RESULTS.md) cover eight actual ArduPlane WASM/EKF + Rust 9-DOF runs.
The baseline comparison uses the same brake-flare hook; seven cases select LQI.
No pitot, canopy-angle truth, external landing RC guidance, or MANUAL handoff.
The normal simulator LQI gains and as-built physics are unchanged.

## Run

From the repository root, with Node 24 and the packaged firmware/physics built:

```sh
python3 research/active/native-landing-2026-10-20/run.py
python3 research/active/native-landing-2026-10-20/assess.py
cd viewer
npm test
npm run build
npx playwright test tests/browser/landing.spec.ts
```

`run.py` uses the local installed Node path; adjust PATH for another machine.
`cases.json` defines calm, 1.5 m/s head/crosswind with 0.2 turbulence, 20%/100%
lidar loss, doubled latency, a long home landing mission, and classical control.
Straight-in cases construct and upload real navigation waypoints using fused
position/course; the long case uses the browser's fixed home landing mission.
Truth is used for initial settling and contact evaluation, never landing control.
Incoming velocity is taken immediately before ground contact, not after the
contact impulse. Mode and ownership assertions cover the entire airborne trace.

`evidence.tar.gz` contains the raw JSON traces; `provenance.json` pins source and
artifacts. Extract traces into `runs/` to rerun the assessment without flying.
The assessment requires AUTO throughout, LQI approach ownership (except baseline),
zero flare throttle, completed playback, sink below 2 m/s and spot error below
25 m. These are experiment acceptance bounds, not flight safety limits.

The mobile browser test checks mission upload, native AUTO display, mode
cancellation/resumption, touchdown, and removal of the fake LAND option.
The firmware also builds for ChibiOS MatekH743, both with default features and
with `AP_PLANE_CUSTOMCONTROL_ENABLED=1` and `AP_PLANE_CUSTOMCONTROL_LQI_ENABLED=1`
in an extra hwdef. The enabled build leaves 118,032 bytes of flash free.
No hardware flight validation.

The simulator stops after near-ground/low-speed fused-state confirmation;
that display-only endpoint has a 0.6 m height tolerance, 0.3 m/s vertical and
2 m/s horizontal thresholds held for 0.25 s. It never forces AP disarm. It uses
mission field elevation if EKF terrain height is unavailable. Canopy collapse
and ground rollout remain unvalidated. Complete lidar loss only demonstrates
the known-flat-field fallback, not arbitrary-terrain landing.

The original three-item fixed landing mission in this evidence is pinned at
micro-agu commit `872bd46`. The current example adds a tangential spiral; see
[landing-spiral-2026-10-20](../landing-spiral-2026-10-20/README.md). Check out the
pinned commit to reproduce the original mission exactly.
