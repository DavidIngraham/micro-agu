# Results

## Joint damping fits

| Assumption | Pitch multiplier | Yaw multiplier | Training objective reduction |
|---|---:|---:|---:|
| nominal | 0.500 | 0.500 | 22.7% |
| slow_wind | 0.500 | 0.614 | 18.4% |
| loose_model | 0.500 | 0.500 | 23.6% |

The search bounds are 0.5–12×. Results near 0.5× are boundary solutions, not identified optima. Objective reductions compare with fixed 3×/3× under the same assumptions; objectives across different discrepancy scales are not directly comparable.

## Held-out forecasts

Arithmetic mean of window RMSE over eight held-out thirty-second windows. Degrees/s for rates. Initial-state inference uses only the preceding ten seconds; the historical baseline uses alpha=0.35 rad, not an average over the three initializations used in the earlier study.

| Damping | Initialization | Initial seconds excluded | Roll rate | Pitch rate | Yaw rate |
|---|---|---:|---:|---:|---:|
| fixed | inferred | 0 | 12.431 | 14.720 | 23.279 |
| fixed | inferred | 2 | 12.140 | 14.484 | 23.124 |
| fixed | inferred | 5 | 12.107 | 14.516 | 22.877 |
| fixed | inferred | 10 | 12.199 | 15.120 | 23.252 |
| fixed | historical | 0 | 13.132 | 17.020 | 23.635 |
| fixed | historical | 2 | 12.157 | 14.517 | 23.115 |
| fixed | historical | 5 | 12.103 | 14.546 | 22.886 |
| fixed | historical | 10 | 12.199 | 15.114 | 23.247 |
| joint | inferred | 0 | 12.692 | 14.310 | 24.018 |
| joint | inferred | 2 | 12.143 | 13.799 | 23.404 |
| joint | inferred | 5 | 12.104 | 13.805 | 23.280 |
| joint | inferred | 10 | 12.207 | 14.305 | 23.812 |
| joint | historical | 0 | 13.321 | 19.734 | 29.101 |
| joint | historical | 2 | 12.159 | 13.836 | 23.389 |
| joint | historical | 5 | 12.100 | 13.840 | 23.285 |
| joint | historical | 10 | 12.207 | 14.300 | 23.807 |

## Initialization disturbance

Mean per-window RMS body-rate difference caused by ±0.1 rad initial pitch-component perturbations, holding wind fixed. Degrees/s, pooled over perturbations and axes within each window.

| Damping | 0–2 s | 2–5 s | 5–10 s | 10–20 s | 20–30 s |
|---|---:|---:|---:|---:|---:|
| fixed | 10.9239 | 0.7078 | 0.1725 | 0.0206 | 0.0006 |
| joint | 27.5742 | 0.7219 | 0.1705 | 0.0200 | 0.0006 |

## Synthetic truth recovery

Wind RMSE is pooled over north/east components, not horizontal-vector magnitude. The mismatch case raises the estimator brake-lift coefficient 30%; the varying-wind case adds 0.3/0.2 m/s sinusoidal north/east variations. These are limited validation cases.

| Case | Wind RMSE m/s | Relative orientation RMSE ° | Pitch multiplier | Yaw multiplier |
|---|---:|---:|---:|---:|
| synthetic-0-fixed-truth | 0.025 | 0.574 | 2.000 | 4.000 |
| synthetic-0-joint-mismatch-truth | 0.081 | 1.132 | 2.180 | 4.134 |
| synthetic-0-joint-truth | 0.026 | 0.598 | 2.219 | 3.984 |
| synthetic-0-joint-varying-truth | 0.081 | 1.183 | 2.248 | 4.068 |
| synthetic-1-fixed-truth | 0.006 | 0.568 | 2.000 | 4.000 |
| synthetic-1-joint-truth | 0.005 | 0.563 | 2.232 | 4.001 |

## Optimization checks

| Case | Objective | Pitch multiplier | Yaw multiplier | Successful termination |
|---|---:|---:|---:|---|
| diagnostic-1 | 889.949 | 0.500 | 0.500 | True |
| diagnostic-8 | 889.928 | 0.500 | 0.500 | True |
| diagnostic-strict | 889.894 | 0.500 | 0.500 | True |
