# Low-speed EKF comparison

Fresh sensorless baseline and adaptive estimator, unchanged zero-trim LQI gains. Sensor reference is the prior campaign, not a fresh paired run.

Tracking: 20–100 s. Late: 150–180 s. Power: 20–180 s. Medians across scenarios except common brake (mean) and pitch oscillation (maximum).

| Configuration | Mode | Complete | TAS error RMS m/s | Common brake % | Max late pitch AC RMS deg | Alt RMS m | Xtrack RMS m | Shaft W | Wh/ground-km |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | AUTO | 8/8 | 0.421 | 13.515 | 0.045 | 3.736 | 4.046 | 72.822 | 4.830 |
| baseline | CRUISE | 8/8 | 1.271 | 10.864 | 0.042 | 1.574 | 0.409 | 68.259 | 4.245 |
| adaptive | AUTO | 8/8 | 0.088 | 15.767 | 0.045 | 3.717 | 4.604 | 74.936 | 5.026 |
| adaptive | CRUISE | 8/8 | 0.261 | 12.498 | 0.040 | 1.559 | 0.543 | 74.164 | 4.690 |
| sensor-reference | AUTO | 8/8 | 0.113 | 1.645 | 0.043 | 3.698 | 4.576 | 70.856 | 4.585 |
| sensor-reference | CRUISE | 8/8 | 0.112 | 0.296 | 0.039 | 1.644 | 0.411 | 68.048 | 4.165 |

## Paired changes

- AUTO estimated_shaft_W: median +0.76%; range -3.07 to +3.60%; improved in 3/8.
- AUTO estimated_shaft_Wh_per_ground_km: median +2.07%; range -7.67 to +7.87%; improved in 3/8.
- CRUISE estimated_shaft_W: median +8.71%; range -7.86 to +14.72%; improved in 3/8.
- CRUISE estimated_shaft_Wh_per_ground_km: median +11.05%; range -9.72 to +30.67%; improved in 3/8.

## Estimator diagnostics

- baseline: late TAS RMS error median/P90/max 0.800/2.727/3.340 m/s; 5/16 runs touch the late speed-integral limit.
- adaptive: late TAS RMS error median/P90/max 0.182/1.289/1.618 m/s; 5/16 runs touch the late speed-integral limit.
- sensor-reference: late TAS RMS error median/P90/max 0.113/0.116/0.117 m/s; 0/16 runs touch the late speed-integral limit.
- Per-case late fusion success fractions: 0: 100%, 1: 100%, 2: 100%, 3: 100%, 4: 100%, 5: 100%, 6: 100%, 7: 100%, 8: 100%, 9: 100%, 10: 100%, 11: 100%, 12: 100%, 13: 100%, 14: 100%, 15: 100%.
- Diagnostic columns: AP milliseconds, core, fusion applied, body air vx/vy/vz, wind N/E, wind variance N/E, beta innovation, threshold TAS. Body air and wind/covariance are post-update; innovation can be stale when fusion is skipped.
