# Filtered-brake trial results

All power numbers are quasi-steady estimated shaft power, not electrical power.

| Trial | Mode | Complete | Alt RMS m | Cross-track RMS m | Max late pitch AC RMS deg | Mean late common % | Median shaft W | Median shaft Wh/ground-km |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| diagnostic-zero | AUTO | 2/2 | 3.522 | 12.726 | 0.030 | 19.419 | 72.752 | 4.759 |
| diagnostic-zero | CRUISE | 6/6 | 2.018 | 0.246 | 0.041 | 6.548 | 67.190 | 4.250 |
| memory-1000 | AUTO | 2/2 | 3.753 | 16.964 | 0.039 | 12.513 | 71.407 | 4.648 |
| memory-1000 | CRUISE | 6/6 | 1.793 | 0.336 | 0.043 | 10.933 | 69.569 | 4.717 |
| memory-3000 | AUTO | 2/2 | 3.568 | 7.631 | 0.039 | 13.511 | 71.480 | 4.681 |
| memory-3000 | CRUISE | 6/6 | 1.283 | 0.000 | 0.036 | 7.731 | 71.413 | 4.968 |

## Screen against fresh diagnostic-zero runs

Tracking gate: each mode median at most 25% higher; every run completes; CRUISE late pitch AC RMS <0.2 deg.
No confidence intervals: startup/estimator states are not identical across runs.

- memory-1000 AUTO altitude_rms_m: +6.6%.
- memory-1000 AUTO xtrack_rms_m: +33.3%.
- memory-1000 CRUISE altitude_rms_m: -11.1%.
- memory-1000 CRUISE xtrack_rms_m: +36.6%.
- memory-1000: screen FAIL.
- memory-3000 AUTO altitude_rms_m: +1.3%.
- memory-3000 AUTO xtrack_rms_m: -40.0%.
- memory-3000 CRUISE altitude_rms_m: -36.4%.
- memory-3000 CRUISE xtrack_rms_m: -100.0%.
- memory-3000: screen PASS.
