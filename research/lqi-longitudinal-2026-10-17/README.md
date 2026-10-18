# Longitudinal envelope campaign

Packaged ArduPlane firmware, EKF airspeed and onboard canopy observer; as-built 132 mm geometry. AI-assisted simulation research.

Candidate includes the normalized-throttle feedback fix from ArduPilot commit 20a8fb59c8. CP3_HUP=0.75 m/s, CP3_HDN=1 m/s; HRTC=0.7 s. Feedback/observer gains, speed target range, 35 deg/s turn limit and 60% nominal throttle ceiling unchanged.

## Results

Candidate: 14/14 passes; all fixed-binary flights including classical comparators: 17/18 passes. Qualified: True. Comparator failures are retained and do not excuse any candidate failure.

| Case | Pass | Up mean m/s | Down mean m/s | Relative pitch RMS / peak deg/s | Quiet altitude RMS m | Shaft W |
|---|---|---:|---:|---:|---:|---:|
| bug-original-limit | FAIL: Error: Initial settling failed | | | | | |
| diagnostic-3-lqi | FAIL: Error: Initial settling failed | | | | | |
| final-0-baseline | False | 0.705 | -1.028 | 5.79 / 43.27 | 0.492 | 82.85 |
| final-0-lqi | True | 0.733 | -1.012 | 2.12 / 11.97 | 0.133 | 79.60 |
| final-1-baseline | True | 0.759 | -1.006 | 6.32 / 45.68 | 0.269 | 88.57 |
| final-1-lqi | FAIL: Error: Initial settling failed | | | | | |
| fix-check-compensation-off | True | 0.499 | -0.504 | 0.99 / 8.38 | 0.613 | 75.23 |
| fix-check-old-limit | True | 0.496 | -0.500 | 0.98 / 6.19 | 0.061 | 75.26 |
| fix-check-voltage-step | True | 0.703 | -1.013 | 1.77 / 16.35 | 0.071 | 85.38 |
| fixed-final-0-baseline | False | 0.705 | -1.027 | 5.64 / 42.53 | 0.470 | 82.82 |
| fixed-final-0-lqi | True | 0.733 | -1.012 | 2.15 / 12.28 | 0.129 | 79.60 |
| fixed-final-1-baseline | True | 0.758 | -1.005 | 5.99 / 44.42 | 0.268 | 88.57 |
| fixed-final-1-lqi | True | 0.760 | -1.015 | 2.40 / 22.02 | 0.302 | 86.18 |
| fixed-fresh-0-baseline | True | 0.677 | -1.035 | 9.18 / 59.07 | 0.509 | 88.38 |
| fixed-fresh-0-lqi | True | 0.666 | -1.076 | 5.96 / 32.56 | 0.228 | 84.40 |
| fixed-fresh-1-baseline | True | 0.744 | -0.976 | 8.18 / 53.74 | 0.335 | 87.78 |
| fixed-fresh-1-lqi | True | 0.690 | -1.007 | 5.37 / 27.19 | 0.238 | 84.78 |
| fixed-heldout-0-lqi | True | 0.703 | -1.020 | 3.66 / 15.77 | 0.035 | 76.22 |
| fixed-heldout-1-lqi | True | 0.707 | -0.980 | 4.26 / 20.20 | 0.026 | 83.69 |
| fixed-heldout-2-lqi | True | 0.647 | -1.035 | 6.60 / 38.73 | 0.197 | 81.02 |
| fixed-heldout-3-lqi | True | 0.637 | -0.961 | 3.74 / 19.28 | 0.167 | 79.77 |
| fixed-heldout-4-lqi | True | 0.680 | -1.053 | 7.44 / 39.35 | 0.253 | 82.69 |
| fixed-heldout-5-lqi | True | 0.678 | -1.092 | 4.93 / 37.38 | 0.247 | 84.18 |
| fixed-regression-0-expanded | True | 0.529 | -0.581 | 3.77 / 16.13 | 0.160 | 74.78 |
| fixed-regression-0-old | True | 0.533 | -0.579 | 3.77 / 18.31 | 0.147 | 74.69 |
| fixed-regression-1-expanded | True | 0.467 | -0.550 | 4.27 / 21.50 | 0.198 | 83.20 |
| fixed-regression-1-old | True | 0.471 | -0.548 | 4.30 / 21.44 | 0.180 | 83.01 |
| heldout-0-baseline | True | 0.665 | -1.064 | 6.39 / 42.38 | 0.091 | 77.92 |
| heldout-0-lqi | True | 0.704 | -1.022 | 3.66 / 15.56 | 0.164 | 76.19 |
| heldout-1-baseline | True | 0.772 | -0.989 | 7.00 / 42.63 | 0.598 | 87.25 |
| heldout-1-lqi | True | 0.708 | -0.980 | 4.26 / 19.99 | 0.150 | 83.69 |
| heldout-2-baseline | True | 0.686 | -1.081 | 9.77 / 49.19 | 0.380 | 85.23 |
| heldout-2-lqi | True | 0.645 | -1.035 | 6.60 / 38.69 | 0.195 | 81.04 |
| heldout-3-baseline | True | 0.670 | -0.981 | 6.75 / 45.65 | 0.084 | 84.05 |
| heldout-3-lqi | FAIL: Error: Initial settling failed | | | | | |
| heldout-4-baseline | True | 0.838 | -1.068 | 10.73 / 66.91 | 0.298 | 88.74 |
| heldout-4-lqi | FAIL: Error: Initial settling failed | | | | | |
| heldout-5-baseline | True | 0.710 | -1.041 | 7.47 / 51.95 | 0.373 | 87.96 |
| heldout-5-lqi | FAIL: Error: Initial settling failed | | | | | |
| nominal-baseline-0.5-0 | True | 0.496 | -0.471 | 4.71 / 40.51 | 0.323 | 78.53 |
| nominal-baseline-0.5-35 | True | 0.528 | -0.466 | 4.74 / 43.03 | 0.373 | 84.81 |
| nominal-baseline-0.75-0 | True | 0.705 | -0.679 | 5.19 / 41.62 | 0.469 | 80.74 |
| nominal-baseline-0.75-35 | True | 0.760 | -0.670 | 5.29 / 43.76 | 0.558 | 86.67 |
| nominal-baseline-1.0-0 | True | 0.973 | -1.034 | 5.55 / 41.67 | 0.527 | 83.33 |
| nominal-baseline-1.0-35 | True | 0.943 | -1.023 | 5.43 / 45.25 | 1.050 | 89.05 |
| nominal-lqi-0.5-0 | True | 0.494 | -0.499 | 0.99 / 6.09 | 0.019 | 75.28 |
| nominal-lqi-0.5-35 | True | 0.501 | -0.499 | 0.95 / 6.34 | 0.065 | 82.06 |
| nominal-lqi-0.75-0 | True | 0.685 | -0.705 | 1.36 / 8.43 | 0.161 | 76.78 |
| nominal-lqi-0.75-35 | True | 0.700 | -0.702 | 1.31 / 8.33 | 0.200 | 83.72 |
| nominal-lqi-1.0-0 | True | 0.979 | -1.011 | 1.93 / 12.19 | 0.157 | 79.98 |
| nominal-lqi-1.0-35 | True | 0.956 | -1.019 | 2.14 / 20.36 | 0.085 | 86.04 |
| regression-0-expanded | True | 0.529 | -0.581 | 3.78 / 17.96 | 0.158 | 74.76 |
| regression-0-old | True | 0.533 | -0.578 | 3.77 / 16.05 | 0.142 | 74.70 |
| regression-1-expanded | True | 0.459 | -0.543 | 4.27 / 20.88 | 0.212 | 82.99 |
| regression-1-old | True | 0.463 | -0.538 | 4.28 / 21.10 | 0.196 | 82.77 |

final-0-baseline: quiet pitch_rms

fixed-final-0-baseline: quiet pitch_rms

## Matched disturbance comparisons

- LQI / PG+TECS pitch_rms: median 0.653, p90 0.681, worst 0.694, n=8.
- LQI / PG+TECS pitch_peak: median 0.529, p90 0.740, worst 0.787, n=8.
- LQI / PG+TECS energy_Wh: median 0.956, p90 0.970, worst 0.978, n=8.

## Voltage feedback defect

The original firmware failed to settle at 20.4 V even with the old +/-0.5 m/s limits. Recorded initialization shows sustained full throttle far above the 80 m target. The custom allocator compared nominal throttle requests with voltage-compensated feedback, introducing a false anti-windup residual. The fix inverses the active voltage scale for allocation/observer/TECS feedback while retaining real limiting and suppression. Motor output itself is unchanged.
Targeted tests cover the old limits at 20.4 V, compensation disabled, and an in-flight voltage change. Original failures and diagnostic initialization traces remain included above. The six matched PG+TECS reference flights use the original binary; that controller does not exercise the changed custom-control feedback. Two additional fresh paired cases use the fixed binary on both sides.

## Interpretation

The 1 m/s climb passed nominal tests but consumed almost the entire throttle ceiling. The selected lower climb bound reserves more power for disturbances. Descent authority has a different actuator margin.
Trim solves are a constrained numerical screen, not proof of feasibility or infeasibility. Initial and flight-seeded grids are retained; actual trajectory speed can differ from the controller target range. No stall/collapse/structural boundary is established.
Stability is assessed separately from tracking and efficiency: bounded attitude/airspeed, pitch response, and weather-off quiet recovery. Finite tests do not prove nonlinear stability. Power is modeled shaft power, not measured battery consumption.
The thermal is a fixed spatial Gaussian; Dryden is seeded. Model coefficients and actuator slew vary in held-out cases. Identical seeds do not force identical wind histories when controllers follow different trajectories.
Raw outputs preserve failures. Existing unrelated controller-design working files were not used as the deployed gain source. The trim Chart and constants are identical to committed design_mimo.py.

The paired comparison gate applies to the candidate; classical-controller recovery failures remain failures in this report. Every LQI case must independently satisfy the absolute gates.

## Reproduce

See REPRODUCE.md for frozen inputs, runtimes, raw traces and commands. Run from the repository: simulation/.venv/bin/python research/lqi-longitudinal-2026-10-17/trim.py. Run campaign.py with the analysis environment and screen, heldout, final, fix-check or fixed; it invokes the real packaged WASM from viewer. protocol.json and the frozen *-specs.json files define the tests.
