# Airspeed sensor comparison

Mode medians except brake (mean of run means) and pitch (maximum). Late=150–180 s; tracking=20–100 s; power=20–180 s.

| Configuration | Mode | Complete | Sensor use % | TAS error RMS m/s | Late common % | Max late pitch RMS deg | Alt RMS m | Xtrack RMS m | Shaft W | Shaft Wh/ground-km |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| diagnostic-zero | AUTO | 8/8 | 0.000 | 0.298 | 12.926 | 0.039 | 3.552 | 5.332 | 73.027 | 4.852 |
| diagnostic-zero | CRUISE | 8/8 | 0.000 | 0.865 | 10.474 | 0.041 | 2.018 | 0.355 | 67.784 | 4.345 |
| eas-zero | AUTO | 8/8 | 100.000 | 0.113 | 1.645 | 0.043 | 3.698 | 4.576 | 70.856 | 4.585 |
| eas-zero | CRUISE | 8/8 | 100.000 | 0.112 | 0.296 | 0.039 | 1.644 | 0.411 | 68.048 | 4.165 |
| memory-3000 | AUTO | 8/8 | 0.000 | 0.423 | 11.727 | 0.126 | 3.682 | 6.639 | 72.836 | 4.885 |
| memory-3000 | CRUISE | 8/8 | 0.000 | 0.382 | 6.875 | 0.041 | 1.041 | 0.000 | 70.923 | 4.650 |
| eas-3000 | AUTO | 8/8 | 100.000 | 0.113 | 1.566 | 0.042 | 3.847 | 4.563 | 70.951 | 4.571 |
| eas-3000 | CRUISE | 8/8 | 100.000 | 0.112 | 0.968 | 0.038 | 1.688 | 0.408 | 67.975 | 4.147 |

## Paired scenario differences

Median of within-scenario percentage changes, not a ratio of group medians. Startup/estimator states differ; these are descriptive, not causal confidence intervals.

- diagnostic-zero → eas-zero, AUTO, estimated_shaft_W: median -2.82%; range -10.81 to -1.22%; lower in 8/8 cases.
- diagnostic-zero → eas-zero, AUTO, estimated_shaft_Wh_per_ground_km: median -4.64%; range -15.77 to -1.22%; lower in 8/8 cases.
- diagnostic-zero → eas-zero, CRUISE, estimated_shaft_W: median -1.14%; range -15.03 to +5.78%; lower in 4/8 cases.
- diagnostic-zero → eas-zero, CRUISE, estimated_shaft_Wh_per_ground_km: median -6.94%; range -21.20 to +2.71%; lower in 5/8 cases.
- memory-3000 → eas-3000, AUTO, estimated_shaft_W: median -2.85%; range -6.14 to -0.31%; lower in 8/8 cases.
- memory-3000 → eas-3000, AUTO, estimated_shaft_Wh_per_ground_km: median -6.03%; range -8.38 to -0.63%; lower in 8/8 cases.
- memory-3000 → eas-3000, CRUISE, estimated_shaft_W: median -3.83%; range -14.65 to +4.54%; lower in 6/8 cases.
- memory-3000 → eas-3000, CRUISE, estimated_shaft_Wh_per_ground_km: median -12.68%; range -23.17 to -5.46%; lower in 8/8 cases.
- eas-zero → eas-3000, AUTO, estimated_shaft_W: median -0.08%; range -0.35 to +0.30%; lower in 5/8 cases.
- eas-zero → eas-3000, AUTO, estimated_shaft_Wh_per_ground_km: median -0.41%; range -0.80 to +0.03%; lower in 7/8 cases.
- eas-zero → eas-3000, CRUISE, estimated_shaft_W: median -0.12%; range -0.69 to +0.11%; lower in 6/8 cases.
- eas-zero → eas-3000, CRUISE, estimated_shaft_Wh_per_ground_km: median -0.21%; range -0.48 to +0.21%; lower in 6/8 cases.

## Sensor and integral diagnostics

- diagnostic-zero: 4/16 runs touched the speed-integrator limit in the late window; maximum absolute late mean TAS error 2.936 m/s.
- eas-zero: 0/16 runs touched the speed-integrator limit in the late window; maximum absolute late mean TAS error 0.014 m/s.
- memory-3000: 8/16 runs touched the speed-integrator limit in the late window; maximum absolute late mean TAS error 2.851 m/s.
- eas-3000: 0/16 runs touched the speed-integrator limit in the late window; maximum absolute late mean TAS error 0.015 m/s.
