# Deployed controller stability and performance screening

This is a paired, seeded engineering sensitivity study of **PG + TECS** versus
the experimental **three-input LQI**, using the packaged ArduPlane WASM, its EKF,
and the Rust external articulated physics. No controller is retuned. Truth is
recorded for analysis, never supplied to the observer as an estimate.

## Protocol fixed before looking at results

- 16 paired scenarios: two nominal cases, then 14 seeded random sensitivity draws.
  Eight AUTO mission cases and eight CRUISE straight-flight cases. These are two
  different tasks and must be reported separately.
- As-built 132 mm riser spacing, existing flight-log calibration, 22.2 V.
  No randomized geometry or invented calibration data.
- Random wind 0–2 m/s, bearing 0–360 degrees, horizontal turbulence RMS 0–0.6 m/s,
  thermal peak 0–2 m/s (60 m radius, 300 m north of launch).
- Independently scale Cmq, joint roll/pitch/yaw damping, CLbrake and CDbrake
  by 0.7–1.3, and brake travel rate by 0.7–1.1. These uniform ranges are
  **assumed sensitivity ranges, not identified probabilities or confidence bounds**.
- Launch with the baseline controller in calm air. After 30 simulated seconds,
  choose AUTO or CRUISE and select the controller. Then: 20 s settling, 80 s
  weather, 80 s calm recovery. CRUISE holds the captured flight reference;
  AUTO continues its mission, so AUTO motion cannot be interpreted as free decay.
- Matched parameter draws and synthetic gust seeds. The EKF/startup phase and
  aircraft paths need not be identical; a spatial thermal produces different
  encountered forcing on different paths. This is pairing, not bitwise replay.
- Sample truth and latest telemetry at approximately 10 Hz in simulation time.
  Standard SIM_SPEEDUP accelerates wall time after assisted launch. No controller
  timestep, EKF bypass, firmware gain or model gain changes.
- Stop and retain partial traces on nonfinite physics, altitude below 3 m,
  roll above 60 degrees, pitch above 45 degrees, or unexpected controller ownership
  more than 5 seconds after selection. These are screening envelopes, not formal
  stability boundaries. Infrastructure errors and launch failures are separate.

## Comparison priorities

1. Completion, envelope violations and controller dropouts, retaining failures.
2. Detrended pitch/roll RMS, body-rate RMS, oscillation spectra and time histories.
   Compare early and late calm recovery in CRUISE; ongoing AUTO turns are forced
   motion, not a damping test. Steady pitch trim is not oscillation amplitude.
3. Altitude and cross-track error, mission progress, actuator endpoint occupancy and effort.
   Cross-track error before waypoint progression is not evidence of good tracking.
   AP navigation altitude targets may differ from LQI's governed reference.

Do not infer robust stability from finite trajectories or a single spectral peak.
Finite-run sensitivity results cannot establish hardware flight safety, model
validity outside the flight-data envelope, or robustness to untested delays,
sensor faults, different battery voltage or unmodeled canopy dynamics.

## Reproduction

With viewer assets/dependencies installed and Node 22+ on PATH:

```sh
python3 research/archive/controller-stability-2026-10-16/run.py --pairs 16 --workers 4
uv run research/archive/controller-stability-2026-10-16/analyze.py
```

Each process stores the scenario, firmware build manifest, telemetry, truth,
actuator history, logs and any failure. `cases/` and `results/` are retained.
Generated reports and figures are research artifacts, not a new deployed tune.

The truth angle/rate and spectrum analysis is checked against a known 1 Hz,
2-degree sinusoid with a nonzero pitch trim and yaw rotation (`test_analysis.py`).
The viewer's 41 unit tests and full TypeScript/Vite build pass. The study adds
seeded weather, adjustable truth sampling and acknowledged SITL pacing to the
reusable harness; default browser behavior stays at its existing settings.

Analysis has PEP 723 dependency metadata for uv; an existing Python environment
with NumPy 2, SciPy and Matplotlib also works. A separate normal-speed replay of
the nominal CRUISE pair is stored in `pacing-check/` and excluded from the 16 pairs.
