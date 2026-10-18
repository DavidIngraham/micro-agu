# Post-hoc performance reassessment

Descriptive analysis of existing traces, not a new acceptance test. No original gates, controller gains or deployment changed. Twelve paired cases at 25 deg/s; six candidate-only cases at 40 deg/s. Wet-flight metrics use t<200 s; steady-turn metrics exclude the first 20 s of each turn. Cases are the statistical units, not individual samples.

| Metric | Reference median | Candidate median | Median paired change | Paired range | Improved cases |
|---|---:|---:|---:|---:|---:|
| mean_bias | 3.625 | 0.600 | -85.0% | -95.0% to -65.5% | 12/12 |
| tracking_rms | 6.678 | 4.889 | -28.7% | -52.8% to -14.4% | 12/12 |
| rate_std | 4.969 | 4.740 | -7.3% | -14.5% to -0.6% | 12/12 |
| pitch_rms | 6.560 | 6.587 | -3.6% | -5.8% to +1.2% | 8/12 |
| pitch_p95 | 12.908 | 12.831 | -0.5% | -8.2% to +1.7% | 6/12 |
| pitch_peak | 23.188 | 27.143 | -0.4% | -24.8% to +44.3% | 6/12 |
| alt_rms | 0.482 | 0.316 | -32.4% | -49.4% to -12.5% | 12/12 |
| alt_peak | 1.188 | 0.923 | -21.9% | -43.1% to +8.5% | 9/12 |
| shaft_W | 81.286 | 78.646 | -2.8% | -5.8% to -0.6% | 12/12 |
| shaft_Wh_air_km | 5.261 | 5.029 | -3.8% | -8.2% to +0.0% | 11/12 |
| common_brake_mean | 0.059 | 0.039 | -50.5% | -86.8% to +29.2% | 9/12 |
| pitch_wind_low | 7.059 | 6.973 | -7.3% | -21.3% to +30.1% | 7/12 |
| pitch_wind_high | 35.506 | 35.526 | -2.4% | -7.0% to +6.9% | 9/12 |

## Interpretation limits

- All 30 analysed flights completed without the harness operational guards firing. Those guards are engineering screens, not demonstrated aircraft safety limits. No calibrated stall, structural-load or collapse boundary was established by this study. The simulator does not model canopy collapse. Do not label these flights physically safe on this evidence alone.
- The 30 deg/s relative pitch-rate and RMS limits are screening choices; no flight-data or structural justification was found in the study. Original failures remain valid threshold exceedances, not proofs of instability.
- Spectral pitch/wind ratios use total three-axis wind power in 0.05–0.5 and 0.5–2 Hz bands. Units are (deg/s)/(m/s). They are descriptive response ratios, NOT identified transfer functions: closed-loop trajectory and mean-wind direction differ, and short turn windows limit low-frequency resolution.
- Weather seeds and physical parameters are paired, but resulting trajectories and wind orientation are not identical. No causal frequency-response or formal stability claim follows.
- Shaft power is modeled, not battery consumption. Mean common brake is not itself an efficiency measure.
- Existing summary.json columns are MEDIANS across cases. Earlier prose calling them averages was inaccurate. This report labels all aggregations explicitly.
- Forty deg/s has no matched reference at that rate in these disturbed cases; it cannot be declared a comparative improvement.

## Quiet recovery and observed envelopes

- reference25: worst final heading RMS 0.815 deg/s; relative pitch RMS 0.088 deg/s; altitude RMS 0.232 m. Settled-window ends after weather removal: [12.0, 12.0, 12.0, 11.0, 14.0, 23.0] s. Minimum airspeed 2.76 m/s; maximum absolute roll 30.5 deg; pitch 23.4 deg.
- candidate25: worst final heading RMS 0.512 deg/s; relative pitch RMS 0.083 deg/s; altitude RMS 0.212 m. Settled-window ends after weather removal: [11.0, 11.0, 13.0, 12.0, 12.0, 23.0] s. Minimum airspeed 2.72 m/s; maximum absolute roll 27.0 deg; pitch 25.5 deg.
- candidate40: worst final heading RMS 0.448 deg/s; relative pitch RMS 0.107 deg/s; altitude RMS 0.231 m. Settled-window ends after weather removal: [12.0, 19.0, 13.0, 13.0, 14.0, 12.0] s. Minimum airspeed 3.02 m/s; maximum absolute roll 47.8 deg; pitch 22.5 deg.

## Individual pitch peaks

| Case | Reference deg/s | Candidate deg/s | Change |
|---|---:|---:|---:|
| 100 | 11.98 | 17.28 | +44.3% |
| 101 | 27.60 | 34.74 | +25.9% |
| 102 | 17.84 | 19.58 | +9.8% |
| 103 | 16.96 | 15.46 | -8.8% |
| 104 | 15.63 | 15.77 | +0.9% |
| 105 | 52.24 | 45.85 | -12.2% |
| 106 | 39.49 | 37.17 | -5.9% |
| 107 | 34.43 | 31.54 | -8.4% |
| 108 | 18.77 | 22.75 | +21.2% |
| 109 | 15.90 | 19.84 | +24.8% |
| 110 | 49.44 | 37.17 | -24.8% |
| 111 | 36.10 | 35.46 | -1.8% |

## Decision

The original binary screen obscured consistent improvements in tracking, altitude RMS and modeled power at 25 deg/s. It does not establish that the candidate is worse or unstable. Pitch-motion performance is mixed: inspect the individual peak regression before promotion. Recovery after weather removal is comparable, with no evidence of persistent oscillation in these finite tests.
No new post-hoc release threshold is invented. The evidence supports continuing qualification of the candidate at 25 deg/s, not an automatic 40 deg/s release. Before publication, investigate pitch-peak cases and run the deferred controller transitions. A 40 deg/s comparative claim additionally needs matched reference runs at 40 deg/s. Physical safety claims need calibrated limits absent from this study.
