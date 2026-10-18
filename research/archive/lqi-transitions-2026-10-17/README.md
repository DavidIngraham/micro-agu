# LQI transition and sensing study

AI-assisted research, 2026-10-17. See [results](RESULTS.md) for completed outcomes.

**Completed: 93/93 simulation runs**, with all recorded operational gates passed. Training selected a 0.7 s reference filter. Across eight fresh transient cases its median paired score improved 33.5% versus the preceding LQI and 19.1% versus standard control; climb/descent relative pitch-rate peaks fell about 78% versus the preceding LQI, without slower acquisition. Shaft energy per ground kilometre changed +0.07% versus preceding LQI and +1.60% versus standard. Fresh AUTO/CRUISE checks passed the declared regression limits.

The selected controller still recovers from the updraft more slowly than standard (12.7 versus 7.8 s medians). Ideal and stressed relative-angle sensing produce almost the same selected-controller behavior. Full-state truth reduces recovery to 11.4 s on the three training physics cases but does not remove the deficit. This supports addressing disturbance-rejection design next, rather than expecting angle-sensor precision alone to solve it. The truth case also knows air-relative velocity and RPM, so its benefit cannot be attributed solely to canopy proprioception. It is not an optimized performance ceiling.

The [audit](audit.json) verifies actual sensing modes, exact firmware hashes, unchanged physics and restoration of twelve original files. The campaign's source changes and generated runtime remain archived locally; no simulator deployment was changed.

The configurable non-research patch separately passes the WASM build; [its check](portable-filter-check.json) also confirms restoration. The 30 chart fixtures, seeded-sensor reproducibility checks, three selection-guard tests, viewer TypeScript check and repository whitespace checks pass. Applying the portable filter alone does not install the experimental proprioception observer or selected gain artifact.

This follows the [matched-reference campaign](../lqi-balanced-2026-10-17/README.md).
Its `responsive` design is this study's `fixed` baseline. No public deployment is performed.

The selected `smooth07` candidate changes only reference shaping; its feedback gains and observer are the same as that prior responsive LQI. The improvement therefore demonstrates a command-transition deficiency, not a measured increase in feedback stability margins. The [optional filter patch](reference-filter.patch) exposes `CP3_HRTC` while preserving its default of zero; it also includes the preceding study's configurable rate limits. Research runs use 0.7 s, ±1 m/s limits and a 2 s altitude approach time constant.

Logged decomposition reconstructs requested actuator output to below 1e-4 normalized fraction. At the nominal climb transition, reference terms contribute approximately +0.17 throttle and +0.10 differential brake before state feedback and allocation. The [observer-error comparison](observer-errors-compared.json) distinguishes estimated canopy states from true canopy states. EKF3's `AP_NavEKF3_Outputs.cpp:getWind` sets vertical wind to zero; LQI's velocity-minus-wind measurement therefore contains vertical-wind error in the updraft. This is a known limitation, not by itself proof that it causes the remaining recovery deficit.

## Questions and interventions

1. Reconstruct the reference, state feedback, integral, brake-memory and allocation contributions at command onset from recorded LQID. Reconstruction residuals must be small; do not attribute the entire actuator change to the observer.
2. Filter the commanded climb rate with 0.3, 0.7 or 1.2 s first-order time constants, integrating that same filtered rate into the altitude reference and using it in both state-reference and actuator feedforward. The zero-time-constant baseline is algebraically unchanged. The filter resets on engagement.
3. Compare the 0.7 s design with higher observer process noise and with doubled pitch-rate cost. All training candidates keep 0.5° angle noise and 40 ms sensor delay.
4. Freeze selection on the three existing training physics cases. Validate on eight new physics seeds (60–67), with eight transient and eight mission runs for standard, prior LQI and selected LQI.
5. Diagnose sensing using the same fixed/selected controllers: ideal angle observer, noisy zero-delay observer, 2°/80 ms stress sensor, and correctly mapped complete state truth. These runs do not select gains.

The state-truth diagnostic uses the exact 16-state synthesis chart, including trim-relative orientation, absolute canopy rates in canopy axes, air-relative body velocity, RPM scaling, and height origin. Thirty independent SciPy fixtures cover quaternion sign, heading, wind and perturbations. Navigation and speed/turn integrators still use ArduPilot's EKF and pitot; this is a diagnostic of inner-state estimation, not an absolute optimum or a flight-ready sensor configuration.

Reference smoothing uses `alpha = DT / (tau + DT)` at the existing 50 Hz controller update. Away from the rate limits, the continuous approximation is `tau*T*h_ref'' + T*h_ref' + h_ref = h_command`, where `T=2 s`. Increasing the smoothing constant changes the approach dynamics as well as the initial actuator jump; it is not automatically better. The 1.2 s candidate was rejected on descent loss despite its lower aggregate score. The selected 0.7 s candidate retains the existing feedback and observer poles; finite scenario outcomes do not establish new stability margins.

## Selection

The previous four task losses and energy/actuator terms remain. The motion subscore now averages two gust-rate RMS penalties (15°/s scale) and two command-rate peak penalties (30°/s scale). Weights are task 65%, motion 20%, actuator 10%, shaft energy 5%. Rank 50% mean plus 50% worst training score, require at least 5% improvement, no operational failure, no task/energy median regression above 20%, and no climb acquisition time regression above 10%.

This prevents slowing acquisition substantially to win on motion. It does not make the trajectories identical. Per-task results and failed/censored cases remain visible. Cases use the existing as-built 132 mm geometry and archived linearization; no new flight-data calibration is inferred.

## Reproduction and provenance

Run `campaign.py` with the ArduPilot Python environment; it imports the preceding campaign's build/run primitives. Each build archives full source, exact WASM and hashes. Original files are snapshotted and restored with concurrent-edit checks. Then run `report.py`, `audit.py`, `test_selection.py`, and the TypeScript `sensor.test.ts` after generating its fixtures with `check_chart.py`. The campaign refuses to overwrite snapshots or output cases.

`failed-build/` preserves an initial compiler rejection (float equality in research-mode parsing), with verified restoration; no flights from it enter results. All artifacts are local research. Optional truth channels use the existing JSON socket transport, not a new WASM ABI.

## Limits

One local linearization, unconstrained LQI synthesis, unilateral brake allocation and shared physics constrain interpretation. No formal gain/phase or nonlinear stability margin is established. The standard controller has different lateral reference limits and does not actively regulate airspeed in its paraglider throttle branch. Shaft Wh per ground kilometre is not battery energy or endurance. The observer tuning sweep does not exhaust delay-aware or nonlinear estimator designs.
