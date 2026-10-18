# Landing refinement with the complete mission controller

This study replaces the published aggressive-preset comparison with a controlled 4 Hz versus 1 Hz measurement-filter comparison. Both use the complete mission-controller-candidate longitudinal parameters and the established lateral FF/P/I/roll damping coefficients. Earlier diagnostics remain in lateral-landing-2026-10-13.md; they are not the baseline of this study.

## Configuration and sampling

Raw data: Mission Planner/analysis/landing-refinement-2026-10-13. Twelve established environmental configurations are run twice, as landing-4hz-00 through 11 and landing-1hz-00 through 11. Each native flight starts a fresh SITL process. The full PG_TURN, TECS, landing, throttle and voltage-compensation settings are audited from PARM; matching experiments differ only in PG_TURN_FILT. PGFL audits verify flare entry and output limits. Truth heading rate is reconstructed from payload quaternion and world angular velocity, independently of the measurement filter.

The final validation set uses seed 2026101307 with twelve independently drawn configurations. The 1 Hz candidate and complete configuration were fixed before inspecting that set. Environmental ranges and flare settings are unchanged. The matched set is a comparison set, not fresh validation. Replay condition 1 remains fixed from the earlier publication; no outcome-based selection is used.

## Local model

Nine relative equilibria cover straight sink rates 0/0.5/1 m/s, ±10 deg/s turns at 0.5/1 m/s sink, and 15%/30% common brake at 1 m/s sink. All balance below 2e-10 maximum mixed-unit residual. Powered partial-brake equilibria do not represent the moving idle-throttle flare. The model retains opposite mixer branches, unequal 1300/1500 microsecond servo strokes, the 55 mm drum, the motor state, live airspeed scaling, and admissible integral trims.

The native controller is 50 Hz, confirmed by SCHED_LOOP_RATE=50 and 20 ms PGYR intervals. Linear plants advance and hold commands over 20 ms with 1.25 ms physics substeps. This corrects the earlier exploratory use of the 400 Hz physics frame period as the controller update. The 16 plant states plus two measurement filters and integral state omit the outer navigation, estimator and longitudinal loop. The neutral absolute-heading pole is excluded from the dominant-pole statistic. Command delays are separate sensitivity assumptions, not a measured servo model.

At zero additional delay, both configurations have decaying non-neutral poles at all nine equilibria. At 0.5 m/s sink and +10 deg/s turn, the 4 Hz controller has a growing mode at 80 and 120 ms added delay; at 120 ms it is +0.02175/s at 1.9983 Hz. The 1 Hz controller has decaying modes throughout the sampled 0/40/80/120 ms delays. Independent nonlinear small-perturbation response checks at 120 ms reproduce the local predictions with relative error below 1.5e-5. This is internal model verification, not real-aircraft validation.

![Landing envelope](../../../docs/assets/landing-refinement/landing-envelope.png)
![Plant frequency response](../../../docs/assets/landing-refinement/landing-bode.png)

## Matched native results

All 24 flights completed and passed parameter audits. Actual heading-rate RMSE improved in nine of twelve conditions; normalized command total variation improved in all twelve. Median command variation fell from 0.08136 to 0.02474 per second. Worst tracking RMSE across the set fell from 29.60 to 10.61 deg/s. Miss improved in six conditions; median miss was 0.998 versus 0.985 m. This is evidence for reduced steering activity, not uniform spot-accuracy improvement. In previously identified condition 2, rate RMSE fell from 22.95 to 5.95 deg/s. Replay condition 1 remains gentle: 0.54 versus 0.51 deg/s, with miss 0.558 versus 0.585 m.

![Matched response](../../../docs/assets/landing-refinement/landing-control-comparison.png)

## Reproduction

Archived scripts and exact configurations are in scripts/landing-refinement-2026-10-13. run.py executes the established matched conditions; run_validation.py executes the new seed and missing 4 Hz crosswind regression. landing_envelope.py and plot_envelope.py regenerate local models and figures. Repository research/archive/legacy-tools/report_landing_refinement.py audits native logs and builds final-only statistics, figures and bounded replay excerpts. research/archive/legacy-tools/report_lateral_missions.py accepts --crosswind-baseline to include the additional mission reference.

Run scripts use the existing WSL environment and absolute raw-data paths; adjust those roots for another checkout. Raw flight logs and full truth traces remain in Mission Planner. The main controller-design article is unchanged from its restored pre-landing version. AI-assisted implementation, analysis and publication.

## Fresh validation results

All twelve fresh runs reached the modeled ground plane, and all twelve finished within 5 m of the target.

| Metric | Final configuration: fresh validation |
|---|---:|
| Within 5 m of target | **12 / 12** |
| Median miss | **1.84 m** |
| 90th-percentile miss | **2.55 m** |
| Worst miss | **2.60 m** |
| Median downward speed at crossing | 1.37 m/s |
| 90th-percentile downward speed | 1.43 m/s |
| Median ground speed | 2.96 m/s |

These values describe the new seed only and are not a before/after estimate of landing accuracy.

## Complete-profile regression and publication

The new 4 Hz baseline passed the native 2 m/s eastward-crosswind AUTO mission. Right/left loiter truth-rate RMSE is 2.286/3.710 deg/s versus 1.835/3.265 for the previously recorded complete-profile 1 Hz run. Maximum corner overrun is 0.716 versus 0.644 m. Previously audited calm and gust missions are reused because the selected controller is unchanged. In gusts, the known left-loiter rate-error increase remains visible in the published comparison.

The default landing loader now selects the complete mission JSON when no explicit study overrides are provided. A native smoke flight with no PG_GAINS or PG_CONTROLLER_CONFIG override passed and logged all 32 configured parameters correctly. Explicit archived-study overrides retain their earlier behavior. Commit 377cd08561 records this setup change; the compiled vehicle and physics binaries are unchanged.

At the user's request, the two final applet illustrations use condition 1 with horizontal gust amplitude raised to 0.25 m/s per component, vertical amplitude 0.0625 m/s. They share one preview-config.json and are excluded from both statistical sets. Wind, throttle, applied brake and velocity are recorded from those native flights, not added as visual motion. The earlier no-extra-gust condition-1 statistics above refer to the matched study.


## Calm and gentle-turbulence illustrations

The published applets now compare environments with the same final 1 Hz controller. A new native run, preview-calm, uses the preview geometry and aircraft configuration with zero mean wind and zero gust amplitude. The existing preview-1hz recording supplies the gentle-turbulence case (1.51 m/s mean wind, 0.25 m/s horizontal gust amplitude). Both are excluded from the statistical sets. The earlier 4 Hz preview remains archived but is no longer embedded in the article.

The original mission gains already work well. Both filters are locally stable at all nine trims without added delay. The linear study explains a delay-sensitive mode and supports a bandwidth refinement; the nonlinear matched runs supply the practical evidence. Median spot error is essentially unchanged (1.00 versus 0.98 m), despite reduced command activity in every matched case and improved truth-rate tracking in nine. The fresh validation set evaluates the chosen configuration, not superiority over the baseline.
