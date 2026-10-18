# Results

32 runs, 32 completed without a screening stop.

Medians below exclude incomplete runs; failures are listed separately.

| Task | Controller | Complete | Disturbed pitch AC RMS ° | Pitch rate RMS °/s | Altitude RMS m | Cross-track RMS m | Late calm pitch AC RMS ° |
|---|---|---:|---:|---:|---:|---:|---:|
| AUTO | baseline | 8/8 | 4.053 | 12.932 | 1.863 | 4.239 | 0.036 |
| AUTO | lqi | 8/8 | 3.990 | 11.969 | 3.822 | 5.790 | 0.036 |
| CRUISE | baseline | 8/8 | 4.662 | 16.684 | 3.017 | 0.405 | 0.037 |
| CRUISE | lqi | 8/8 | 4.232 | 12.370 | 1.651 | 0.450 | 0.037 |

## Lateral motion and command effort

Medians during disturbances. AUTO includes commanded turns. Travel is total
absolute command change per second, in normalized actuator fractions.

| Task | Controller | Roll AC RMS ° | Roll-rate RMS °/s | Yaw-rate AC RMS °/s | Throttle travel/s | Left travel/s | Right travel/s |
|---|---|---:|---:|---:|---:|---:|---:|
| AUTO | baseline | 2.937 | 7.109 | 8.554 | 0.058 | 0.035 | 0.119 |
| AUTO | lqi | 2.638 | 6.933 | 7.520 | 0.016 | 0.037 | 0.025 |
| CRUISE | baseline | 1.808 | 9.219 | 7.175 | 0.078 | 0.048 | 0.133 |
| CRUISE | lqi | 1.426 | 8.096 | 6.501 | 0.016 | 0.027 | 0.020 |

## Failures

None. Completion alone does not mean acceptable damping.

## Paired differences (LQI minus baseline)

Only complete pairs. Bootstrap intervals resample scenarios within task; with eight
or fewer pairs they are exploratory, conditional on these assumed ranges.

- AUTO pitch_ac_deg: median -0.048, 95% paired bootstrap [-0.334, 0.039], n=8.
- AUTO pitch_rate_deg_s: median -1.435, 95% paired bootstrap [-1.684, -0.513], n=8.
- AUTO altitude_rms_m: median 1.275, 95% paired bootstrap [-0.532, 2.434], n=8.
- CRUISE pitch_ac_deg: median 0.071, 95% paired bootstrap [-1.569, 0.462], n=8.
- CRUISE pitch_rate_deg_s: median -1.171, 95% paired bootstrap [-6.295, -0.094], n=8.
- CRUISE altitude_rms_m: median -0.813, 95% paired bootstrap [-2.325, 0.020], n=8.

## Individual CRUISE recovery

Linear-detrended truth pitch over 105–135 s and 150–180 s. A ratio near one
at appreciable amplitude suggests persistent motion; it is not a pole estimate.

| Case | Controller | Early RMS ° | Late RMS ° | Late/early | Late spectral peak Hz |
|---|---|---:|---:|---:|---:|
| 1 | baseline | 6.095 | 5.812 | 0.954 | 0.508 |
| 1 | lqi | 0.179 | 0.035 | 0.197 | 0.391 |
| 3 | baseline | 0.241 | 0.026 | 0.110 | 0.391 |
| 3 | lqi | 0.238 | 0.041 | 0.174 | 0.391 |
| 5 | baseline | 0.039 | 0.034 | 0.867 | 0.352 |
| 5 | lqi | 0.055 | 0.038 | 0.698 | 0.273 |
| 7 | baseline | 0.205 | 0.040 | 0.193 | 0.547 |
| 7 | lqi | 0.141 | 0.025 | 0.177 | 0.352 |
| 9 | baseline | 0.547 | 0.035 | 0.064 | 0.430 |
| 9 | lqi | 0.201 | 0.027 | 0.135 | 0.391 |
| 11 | baseline | 0.077 | 0.037 | 0.481 | 0.547 |
| 11 | lqi | 0.270 | 0.036 | 0.133 | 0.234 |
| 13 | baseline | 0.087 | 0.037 | 0.422 | 0.508 |
| 13 | lqi | 0.217 | 0.041 | 0.191 | 0.273 |
| 15 | baseline | 5.831 | 5.728 | 0.982 | 0.508 |
| 15 | lqi | 0.479 | 0.039 | 0.082 | 0.352 |

Actuator endpoint occupancy and total travel are in metrics.json. A released
brake is normal for the baseline mixer; endpoint occupancy is not by itself
evidence of controller saturation. The study does not record unconstrained demand.

## Normal-speed check

- baseline: complete=True, late pitch AC RMS 5.824 deg, pitch rate RMS 20.047 deg/s, peak 0.508 Hz.
- lqi: complete=True, late pitch AC RMS 0.015 deg, pitch rate RMS 0.042 deg/s, peak 0.352 Hz.
