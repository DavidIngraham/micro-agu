# Landing guidance measurements

One run per condition; incoming values immediately above the model contact surface.
Stock uses an LQI-selected AUTO approach and classical LAND. Guided variants use LQI in CRUISE, then an external research flare through the manual mixer.

| Case | Finished | Spot error (m) | Sink (m/s) | Ground speed (m/s) | Pitch (deg) | LQI approach active |
|---|---|---:|---:|---:|---:|---:|
| bias-high-feedback | True | 2.64 | 1.29 | 3.97 | 13.22 | 100% |
| bias-high-feedforward | True | 1.95 | 1.31 | 4.36 | 11.95 | 100% |
| bias-low-feedback | True | 2.51 | 0.82 | 2.85 | 19.21 | 100% |
| bias-low-feedforward | True | 2.90 | 0.90 | 2.83 | 16.74 | 100% |
| calm-feedback | True | 0.43 | 0.81 | 3.25 | 18.33 | 100% |
| calm-feedforward | True | 1.11 | 0.79 | 3.27 | 18.32 | 100% |
| calm-stock | True | 9.75 | 1.39 | 4.25 | 2.33 | — |
| cross-feedback | True | 5.47 | 0.78 | 3.41 | 17.18 | 100% |
| cross-feedforward | True | 4.81 | 0.79 | 3.07 | 16.07 | 100% |
| cross-stock | True | 3.64 | 1.02 | 4.32 | 8.74 | — |
| head-feedback | True | 4.84 | 0.94 | 2.66 | 16.06 | 100% |
| head-feedforward | True | 5.10 | 0.90 | 2.58 | 14.45 | 100% |
| head-stock | True | 1.72 | 1.51 | 2.00 | -0.76 | — |

Failures: []

These results separate position accuracy from severity; reaching the boundary is not a soft-landing pass.
