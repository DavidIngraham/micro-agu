# Scheduled state-feedback extension — 15 October 2026

AI-assisted controller research and numerical experiments. No firmware, production settings, or hardware are changed. The published narrative is in [the controller article](../../../docs/controller-state-space.md). The [original fixed-gain LQI experiment](../state-space-2026-10-14/README.md) remains archived; its tables must not be combined with this comparison.

## Question and experimental boundaries

Can full-state feedback offer useful improvements after restoring voltage/airspeed compensation, operating-point feedforward, reference shaping and actuator handling? Compare retained conventional gains, the previously retuned conventional gains, and scheduled LQI under a **new shared test harness**. This is not a native autopilot reproduction or an isolated experiment on extra state information: feedback, feedforward and filtering still differ. Existing native AUTO passes apply only to the earlier conventional-controller study.

The physics checkout remains `c44434dee6d68340fa463c0341d245f77451cd19`. `verify.py` checks all original model hashes. The simulator and source dependency paths are in `experiment.py`; no sibling repository is modified. The initial common coordinate chart is from the original mission/level plant.

## Controller

- Eighteen mission-profile trims: voltage [20.4, 22.2] V, climb [-0.5, 0, 0.5] m/s, turn [-10, 0, 10] deg/s. Both turn directions retain the actual brake mixer and asymmetric rigging. States, commands and Jacobians are computed in a common chart. All trim residuals are below 1e-6; checks and drift are retained in `grid.json`.
- Scheduled controller has sixteen physical states and two integrators (height and heading-rate errors), with direct command output. It no longer uses the old LQI's command-increment memory states.
- Trilinear interpolation of trim state, trim command and LQI gains over voltage, requested climb and requested turn. Scheduling coordinates are clamped to the grid, not extrapolated. No nonlinear trim solver runs during simulation.
- Control uses `u_ref - diag(US * scaling) K [(x-x_ref)/XS, integral]`. Scaling is `[22.2/voltage, clip(5/max(airspeed,2), .25, 2.5)]`. The same scaling is included in the design input matrix, so it is not an extra multiplier added to gains already designed in physical actuator coordinates. Feedforward trim commands are already physical commands and are not scaled again.
- Airspeed is an actuator-effectiveness correction, not a separate gain-grid axis. Natural speed changes with climb/turn are represented in the trim grid; independent speed perturbations get the live effectiveness correction and velocity-state feedback. This is not an independently commanded airspeed controller.
- Full-state feedforward uses feasible steady climb/turn equilibria. It does not include trajectory-acceleration feedforward. The old conventional climb/descent and turn feedforward are retained for the conventional families; they do not receive oracle equilibrium trajectories.
- All families receive the same reference shaping: height first-order lag 0.5 s, vertical reference slew ±0.5 m/s, turn first-order lag 0.1 s, ±10 deg/s turn and ±40 deg/s² turn acceleration. These deliberately keep scheduled reference conditions inside the grid. Limits-recovery requests exceed this envelope before shaping.
- All families use the same output layer: throttle ceiling `min(.6*22.2/V,1)`, total-throttle slew of that ceiling per second, brake clipping ±1, physical asymmetric strokes, and 221.522558906 mm/s brake travel. The continuous voltage ceiling replaces the earlier harness's integer-percent rounding.
- Both families use applied-command back-calculation with a 0.25 s tracking time, including the travel-averaged brake command and post-slew throttle. Conventional integral contributions are capped at ±0.3 throttle-equivalent and ±0.15 brake; scheduled normalized error integrals at ±20. Feedback/anti-windup coordinates differ, but physical limits and tracking time are shared.
- This shared total-throttle slew, reference envelope and back-calculation differ from native TECS's P-plus-feedforward limiting and conditional integrators. Therefore the new baseline is not numerically identical to the old baseline. It is labeled conventional baseline in the **new harness** throughout the article.

No launch, flare, autonomous-navigation transition, sensor-failure or controller-handover validation is claimed. Initial controller memories are consistent with the scenario's trim. The voltage-ramp run changes voltage continuously without resetting those memories.

## Design and selection

`screen` evaluates 45 weight combinations: pitch/relative-pitch cost multiplier [.3,3,30], integral cost [.03,.3,3], and input penalty [1,10,100,1000,10000]. State diagonal regularization is .002 in engineering-normalized coordinates. Each combination produces eighteen locally designed gains.

The score combines linear gust response and recovery from initial height/heading-rate offsets. Each component is normalized by its median over the declared family, then combined as half average and half worst operating-node cost. This differs from the original command-step objective; scores across the two studies are not directly comparable. Seven reporting outputs retain the original engineering scales. The gust spectrum is unchanged, with a new held-out phase realization for nonlinear tests.

An initial 27-combination screen selected a design with 8 unstable points in a 288-case combined authority/delay check. `initial-screen.json` and `initial-verification.json` retain this rejected diagnostic. The expanded screen admits only candidates stable at every such point. The selected weights are motion 30, integral 3, effort 1000. No nonlinear result was used in the selection objective.

The 288 design-screen cases are 18 trims × independent throttle/brake authority [.75,1.25] × independent added throttle/brake delays [0,40] ms. These are selection checks, not held-out validation. A separate 375-point interpolation check samples 5 voltages × 5 climbs × 5 turns × [.75,1,1.25] brake-scaling multipliers. It uses interpolated plants and gains; it is not proof of stability for arbitrary scheduling trajectories or a newly linearized physical plant at every intermediate point. Both sets are locally stable for the selected controller.

Four independent central-difference closures run the actual controller, physical integration and actuator handling around distinct nodes. Their relative matrix errors are below 6e-9. These catch update-order and scaling inconsistencies between synthesis and simulation. Separate assertions verify convex schedule weights and actuator limits/slew.

## Nonlinear comparisons

All runs last 80 s with 20 ms controller updates and sixteen integration substeps. Each scenario/controller starts independently; wind realization and requests match within a scenario. `results.json` contains 24 truth-sensed runs:

| Scenario | Change from nominal level hold |
|---|---|
| maneuver | 1 m height request ramped over 2 s; 10 deg/s turn from 20–45 s with 2 s ramps, then shared shaping |
| gust | New phase seed 2026101501, component RMS [.5,.5,.25] m/s |
| strong_gust | 1.5× amplitudes; seed 2026101502 |
| soft_roll_gust | Held-out dynamic-roll physics, same nominal gust seed |
| low_voltage_gust | 20.4 V constant, same nominal gust seed |
| voltage_ramp | 22.2→20.4 V over 15–45 s with continuous gusts, seed 2026101503; no state/trim reset |
| speed_transition | Climb then descent, height ramps at ±.5 m/s over 10–25 and 35–50 s; 8 deg/s turn request |
| limits_recovery | 10 m altitude request at 10–30 s; 25 deg/s turn at 15–30 s, clipped by reference governor; gusts end at 50 s for calm recovery |

Metrics cover 5–80 s for maneuver, transition and recovery, and 15–80 s for gust scenarios. Height/heading errors use the **shared shaped reference**, not the unshaped request. Tail height RMS covers 70–80 s. This differs from the old study's windows. `limited_fraction` counts any difference >1e-6 between requested and applied commands, including ordinary travel averaging; it is not a hard-saturation fraction. `ceiling_fraction` specifically records the throttle ceiling. Headroom is physical available throttle fraction.

`ablation-results.json` contains four paired ablations: disable voltage scheduling/compensation (freeze the controller's voltage at 22.2 V, retaining the actual voltage-dependent physical limit), or disable live speed correction (retain its reference-node value). Gains are unchanged. Cases are voltage ramp and speed transition. The constant-voltage speed-transition voltage ablation is a useful negative control and exactly matches the complete controller. Voltage compensation improves ramp height RMS from .4277 to .3781 m; the speed correction changes these particular metrics very little. This does not establish that speed scheduling is unnecessary outside the tested envelope.

## Estimation extension

`observer.py` supplies an illustrative nonlinear predictor with scheduled linear covariance propagation and Kalman measurement updates. It estimates the sixteen physical states plus three constant-wind/random-walk states. It receives no canopy gyro, rotor RPM, or true-wind measurement. It starts with small hidden-state errors (canopy attitude [2,-2,1] deg and canopy rates [1,-1,1] deg/s).

Payload attitude and body gyros update at 50 Hz; navigation velocity and height at 5 Hz. Independent Gaussian sample noise is 1 deg attitude, 1 deg/s gyro, .1 m/s velocity and .2 m height. Normalized process covariance per prediction is diag([1e-5]×16 + [.01]×3); initial covariance .1 I. Prediction uses the known mission/dynamic-roll physics for each case, applied commands and measured voltage. Rotating the estimated wind into the new heading-aligned chart accompanies prediction. Covariance Jacobians are scheduled from the nominal mission grid.

This is an optimistic known-model sensor study, not a validated flight EKF: heading alignment is ideal, bias/correlated AHRS errors and transport delays are absent, and the illustrative noise parameters have not been flight-identified. Both conventional families also receive the same observer outputs and estimated airspeed, so this is a matched sensor-input comparison, not native-estimator validation.

`observer-results.json` contains twelve runs: three families × maneuver, gust, soft-roll gust and voltage ramp. All remain finite, but scheduled LQI is more sensitive to estimation. In the voltage-ramp case, its height RMS is .4930 m versus .4437 m conventional baseline; the truth-state advantage does not survive. Further observer/control co-design is required.

## Second iteration: bounded state-feedback correction

`iterate.py` preserves the first-pass experiment and adds two development searches. The first tests 9 hidden-state attenuation / pitch-damping variants (hidden feedback fraction [.25,.5,1], added pitch damping [0,.05,.1] through two 1.5 Hz stages). The second tests 15 composite variants: correction blend [.1,.2,.3,.5] × cutoff [.2,.5,1] Hz, plus three .3/0.5-Hz variants with extra pitch damping [.02,.04,.08]. Some initial smaller searches were expanded within development; the final files retain the complete declared families.

The composite controller is the existing conventional baseline plus a first-order-filtered `clip(blend*(u_LQI-u_base),-.05,.05)` correction, independently bounded on each actuator. It retains the conventional inner-loop filters and voltage/speed handling, with scheduled trim/feedforward in the LQI path. Anti-windup gives the conventional path the actual actuator discrepancy, excluding the intentional correction; the LQI path tracks the final applied command. This is a hybrid augmentation, not an unconstrained LQI optimum or a pure controller replacement.

Every candidate is screened on the 288 authority/delay cases before nonlinear development. For composite designs, the actual controller memory update is differentiated at an equilibrium, including both sets of integrators and the correction filter; this is a 28-state closure. Nonlinear development uses maneuver, gust and voltage ramp under truth and observer conditions. Criteria are fixed: every case's height and pitch RMS <=105% of conventional baseline, heading RMS <=110%. The objective weights height .4, pitch .35, heading .25, then combines half average and half worst case.

**No design passed all development criteria.** The retained diagnostic is blend .3, cutoff .5 Hz, no extra damping; it is selected by smallest worst constraint violation, then score. It passes local authority/delay screening, worst pole −.02435/s, but observer-based maneuver pitch RMS remains about 6.5% above baseline. The criteria are not loosened and this design is not recommended for deployment.

Validation freezes that choice, increments every phase/noise seed by 1000 and multiplies gust amplitudes by 1.2. Calm truth-state maneuver/transition runs are regressions with unchanged references, not new command holdouts. Forty-eight runs compare conventional baseline/candidate, direct first-pass LQI and the bounded correction: eight truth scenarios and four observer scenarios per controller. `iteration2/validation-results.json` records seeds and amplitudes. Large direct-LQI observer regressions are reduced, but the 5% maneuver pitch limit still fails. `iteration2/validation-audit.json` reports this failure explicitly. No parameters were retuned within that validation. The subsequent direct-feedback investigation below is a new development phase with a separate final validation set.

```sh
$PY $STUDY/iterate.py tune
$PY $STUDY/iterate.py hybrid
$PY $STUDY/iterate.py validate
$PY $STUDY/iteration_report.py
```

The searches retain all stable development results, selection criteria and candidate stability outcomes. Each simulation uses a distinct controller identifier so raw traces do not overwrite the first-pass experiment. The held-out report independently recomputes height errors and checks finite trajectories and physical throttle bounds. The first-pass report's 40-run audit and the second-pass 48-run validation audit are separate; exploratory development runs are not counted as independent validation.

## Reproduce the first pass and inspect

```sh
export OPENBLAS_NUM_THREADS=1
PY=/home/david/envs/venv-ardupilot/bin/python
STUDY=research/archive/scheduled-state-feedback-2026-10-15
$PY $STUDY/experiment.py prepare
$PY $STUDY/experiment.py screen
$PY $STUDY/verify.py
$PY $STUDY/run_suite.py
$PY $STUDY/run_observer.py
$PY $STUDY/run_ablations.py
$PY $STUDY/report.py
```

Independent simulations use four worker processes. The three run scripts must complete before `report.py`, which audits all 40 run gain hashes, finite trajectories, throttle bounds and headline metrics from raw traces. `provenance.json` records source hashes. Intermediate individual run JSON files make failures inspectable; aggregate JSON files are the published results. Raw compressed traces are ignored but regenerated by these commands. The three figures are retained for the blog.

## Direct-feedback refinement with accelerometer estimation

`estimation_study.py` diagnoses state-error contributions to throttle and screens process covariance. Vertical velocity dominates the throttle error contribution; canopy-state attenuation alone misses that mechanism. `imu_observer.py` adds payload specific-force measurements at 50 Hz with 0.2 m/s² independent noise. Sensor generation uses a 1.25 ms physics advance and gravity subtraction in the payload frame. True state/wind are used only to synthesize sensor measurements. Prediction uses nominal mission physics for every plant, so softer-roll validation includes unknown plant mismatch. Bias, correlated AHRS errors, heading-alignment error and transport delay remain absent.

`imu_study.py tune` compares six process covariance choices (physical state [0,1e-7,1e-6], wind [.001,.01]) for baseline and direct feedback in three development scenarios. The selected covariance is state 1e-6, wind .001. `codesign` screens nine vertical-velocity/height feedback factors ([.3,.5,.7] × [.25,.5,1]); `refine` tests brake-row factors [1.15,1.3,1.5] at the retained .7/.25 setting. All retain full hidden-state feedback and add voltage-scaled .05 pitch damping through two 1.5 Hz filters. The final brake factor is 1.5. These changes are implemented by `iterate.Shaped`; they are structured LQI-derived feedback, not an unchanged Riccati optimum.

The original per-case acceptance limits remain height/pitch <=1.05 baseline and heading <=1.10 baseline. The selected variant passes all six development scenario/sensing comparisons, with score .903239 and worst normalized constraint .990481. All 288 local authority/delay tests pass. `final_verify.py` independently differentiates actual closed-loop updates at four nodes; maximum relative discrepancy is below 8e-9. This is local evidence, not a global stability proof.

`imu_study.py final` freezes that design and compares baseline, conventional candidate and refined direct feedback across eight scenarios with truth and IMU/navigation estimation: 48 runs. Seeds increment by 3000 from the original definitions, gust amplitude multiplies by 1.2, and maneuver requests change to 1.5 m and 8 deg/s. No final-validation result is used for selection. `final-validation.json` records the conditions and controller parameters; the baseline and candidate also receive the new observer. Final truth traces use the distinct `scheduled_final_validation` identifier to preserve development traces.

`final_report.py` independently recomputes all three headline metrics from raw traces and checks finite values and actuator bounds. Pitch RMS decreases in all 16 direct-versus-baseline comparisons. Observer-based voltage-ramp height RMS improves .4113→.3561 m. However only 7/16 comparisons meet every acceptance limit: gust heading penalties and maneuver/transition height penalties remain. The result is progress on the original deficiencies, not universal superiority or native/flight qualification. `iteration2/final-audit.json` explicitly retains every pass/fail result and source hashes.

```sh
$PY $STUDY/estimation_study.py
$PY $STUDY/imu_study.py tune
$PY $STUDY/imu_study.py codesign
$PY $STUDY/imu_study.py refine
$PY $STUDY/final_verify.py
$PY $STUDY/imu_study.py final
$PY $STUDY/final_report.py
```

Earlier first-pass and bounded-correction results remain historical experiments with their own sensor models and seeds; compare controllers within an experiment. The final validation is separate from both development and the earlier hybrid validation. No firmware controller was installed or flown.

## Interactive comparison

The blog applet uses all 48 frozen final-validation runs. `export_comparison.py OUTPUT.json` exports height error, payload pitch rate, heading-rate error and voltage every fifth sample (10 Hz display), alongside the original full-resolution RMS metrics. It does not rerun or retune the controllers. Generate the website data with `$PY $STUDY/export_comparison.py viewer/assets/recordings/controller-comparison/data.json`. The applet lives in that website repository under `assets/microagu-viewer/`; its source commit identifies the underlying study, not the later presentation changes.

The 3D extension records actual 26-state payload/canopy poses at 10 Hz, with quaternion interpolation for display, rotor RPM, voltage and the previously applied actuator command. Run `export_poses.py viewer/assets/recordings/controller-comparison/poses` with the study Python environment. It replays 32 frozen baseline/direct cases, asserts all three RMS metrics match the retained validation within 1e-10, and checks quaternion normalization. Pose rows contain time, 26 physical state components, RPM, voltage, throttle and two brake travels. No state is reconstructed from plotted rates. Renderer source is `viewer/apps/applets/controller-comparison/applet.ts`; build with `npm run build` in `viewer` and publish the standard page and its referenced shared chunks. It reuses the existing coordinate conversion, payload rigging and CAD asset. Both cameras translate with their own payload while retaining a common orientation and physical scale; the ground grid remains earth-fixed.

The comparison now uses the standard viewer multi-page build (`npm run build`), with entry `viewer/controller-comparison.html`, shared Three.js chunks, and one composited 2D/3D canvas matching the landing applet. Export data and poses to `viewer/assets/recordings/controller-comparison/` before building; publish `dist/controller-comparison.html`, its referenced assets, and `dist/controller-comparison/` under the site’s `assets/microagu-viewer/`. The former standalone URL redirects to this page. The separate library build has been retired. Playback starts on load and on scenario selection, and loops at 80 s; pausing and timeline scrubbing remain available.

Trajectory views use the recorded payload NED positions, transformed with the same `position()` mapping as the aircraft. Each 3D path is translated by the negative current payload position alongside the earth-fixed grid to preserve the following camera convention. Overview plots share bounds between controllers. Top-down is east/north with equal metre scales; side profile is relative height versus cumulative horizontal path length (not a north-plane projection), with independent axis scales. Marker positions interpolate at the shared playback time. No additional simulation or retuning was required.
