# Campaign measurements

## Actual ArduPlane LAND

Incoming values are measured just above contact. All six finishes use classical control.

| Approach | Spot error (m) | Along error (m) | Cross error (m) | Sink (m/s) | Ground speed (m/s) |
|---|---:|---:|---:|---:|---:|
| land-baseline-calm | 7.68 | 7.64 | -0.84 | 1.38 | 4.24 |
| land-baseline-cross | 8.85 | -8.85 | 0.02 | 1.52 | 4.17 |
| land-baseline-head | 9.09 | -8.23 | 3.85 | 1.38 | 3.28 |
| land-lqi-calm | 32.44 | 32.44 | 0.31 | 1.38 | 4.24 |
| land-lqi-cross | 20.25 | 20.25 | -0.39 | 1.53 | 4.43 |
| land-lqi-head | 11.27 | -10.50 | 4.10 | 1.69 | 2.94 |

## Equal pilot-request recovery

Recovery time includes the initial eight-second descent, ten-second climb and five-second settling requirement.

| Controller / turn request | Altitude loss (m) | Peak sink (m/s) | Peak pitch (deg) | Recovery time (s) |
|---|---:|---:|---:|---:|
| matched-input-baseline-0 | 7.09 | 1.16 | 23.56 | 28.10 |
| matched-input-baseline-25 | 8.48 | 1.67 | 21.07 | 29.00 |
| matched-input-lqi-0 | 5.95 | 0.88 | 14.42 | 26.20 |
| matched-input-lqi-25 | 6.05 | 0.94 | 15.01 | 26.25 |

These four runs support further recovery work, not a statistical superiority claim.
LQI remained active throughout these recovery maneuvers.

## Native flare bias sensitivity

1 m indicated-height trigger, 54 mm common brake. Sink perturbation is applied at 3 m true AGL.

| Height bias (m) | Sink disturbance (m/s) | Incoming sink (m/s) | Ground speed (m/s) |
|---:|---:|---:|---:|
| -0.50 | -0.30 | 1.341 | 2.280 |
| -0.50 | 0.00 | 1.392 | 2.252 |
| -0.50 | 0.30 | 1.445 | 2.230 |
| -0.25 | -0.30 | 1.138 | 2.364 |
| -0.25 | 0.00 | 1.186 | 2.305 |
| -0.25 | 0.30 | 1.236 | 2.250 |
| 0.00 | -0.30 | 0.919 | 2.700 |
| 0.00 | 0.00 | 0.959 | 2.607 |
| 0.00 | 0.30 | 1.003 | 2.515 |
| 0.25 | -0.30 | 0.838 | 3.389 |
| 0.25 | 0.00 | 0.836 | 3.279 |
| 0.25 | 0.30 | 0.840 | 3.163 |
| 0.50 | -0.30 | 1.155 | 4.078 |
| 0.50 | 0.00 | 1.140 | 4.001 |
| 0.50 | 0.30 | 1.128 | 3.922 |
