# Controller tuning and state-space study — 14 October 2026

Read [the article](../../../docs/controller-state-space.md). No production controller defaults or firmware sources are changed by this study.

The original fixed-gain LQI benchmark below is retained as historical evidence. The main article now compares [scheduled feedback with practical compensation and a second iteration](../scheduled-state-feedback-2026-10-15/README.md). That extension uses a new shared reference/actuator harness and fresh baseline runs; its metrics are not interchangeable with these results.

The retained result is a tradeoff, not a replacement recommendation. The conventional candidate reduces the nominal coupled sensitivity peak from 6.03 to 2.27; paired nonlinear gust tests show 4.4–6.2% lower height RMS with 5.2–7.9% greater pitch-rate RMS. Full-state LQI makes the commanded climb gentler but performs worse in the tested gusts. All four native baseline/candidate calm/gust AUTO regressions pass; BIN audits confirm the overrides and all backends report zero errors/clipped packets.

## Scope and provenance

The nonlinear model is `ardupilot-high-dof/Tools/simulation/paraglider` at source revision `c44434dee6d68340fa463c0341d245f77451cd19`. `plants.json` stores hashes of the model source, compiled library, parameters, profiles, actuator and baseline controller file. The Micro-AGU starting revision is `a5e0fab`. Other untracked flare/spiral studies present in the workspace were left untouched.

Six straight-flight operating plants: mission level/climb/descent, mission level at 20.4 V, dynamic-roll level/climb. The dynamic-roll climb is held out from optimization. All use Trout Lake mass/rigging; this study does not retune landing, flare, or deep-spiral behavior.

The local plant has 16 states: the previous 15-state internal chart including rotor speed, plus altitude. Absolute horizontal position and yaw are removed. The chart and state order are in `study.py` and `plants.json`. Inputs are physical throttle fraction and differential-brake fraction, on the positive/right-brake trim branch. The wind Jacobian has three NED components. Trims are resolved using the current model, not accepted uncritically from historical caches.

The sampled plant advances at 20 ms with 16 physics substeps (1.25 ms). The 24-state conventional closure updates its filters and integrators before issuing a held command. It freezes airspeed scaling locally, retains voltage compensation and the operating-side throttle feedforward slope, and omits inactive limits. Nonlinear checks restore live speed scaling, the mixer branches, asymmetric physical strokes, piecewise feedforward, saturation and the travel-rate actuator. They use exact simulated measurements rather than an EKF. This is a native-algebra diagnostic, not a complete reproduction of native scheduling, trajectory shaping, or estimation.

## Objective and normalization

The seven cost outputs are height / 1 m, heading rate / 10 deg/s, climb / 0.5 m/s, payload and relative pitch rates / 30 deg/s, throttle / 0.1, and differential brake / 0.1. State diagnostic scales are angles / 10 deg, velocities / 0.5 m/s, angular rates / 30 deg/s, rotor / 1000 RPM and altitude / 1 m. These are declared engineering scales, not mechanical-energy weights.

Command cost is the infinite-time integral of squared output deviations from the new equilibrium after separate normalized altitude and heading-rate steps. The normalization amplitudes do not assert that finite 1 m or 10 deg/s instantaneous steps are within the linear regime. Gust cost sums squared frequency responses at 0.08, 0.13, 0.21, 0.34, 0.55 and 0.89 Hz with weights [1, 1, .8, .6, .4, .25], normalized to unit squared sum, and component RMS [0.5, 0.5, 0.25] m/s. It is the independent-phase ensemble response of an illustrative multisine, not a calibrated turbulence spectrum.

For each plant, the score is half its command cost relative to baseline plus half its gust cost relative to baseline. The optimizer minimizes half the mean score plus half the worst score across five training plants. The held-out sixth plant is evaluated afterward. Differential evolution uses seed 20261014, population multiplier 7, 45 generations and bounded polishing.

`optimization-effort.json` is the first unconstrained result. `optimization.json` is the retained constrained candidate. The latter prevents increases in the altitude and heading integrated squared-error components or aggregate gust cost on training plants, and requires dominant-pole decay at least 80% as fast as baseline. A 1e-6 feasibility tolerance is used. It reached the iteration limit; the retained point is feasible, not a globally optimal or fully converged solution. The step cost includes both reference channels, and its seven components are retained so aggregate improvements remain inspectable.

## State benchmark and measurement diagnostics

`oracle_screen.py` screens 72 discrete LQI weight combinations. It designs at nominal level trim, uses one gain across all plants with trim offsets, and selects using the same five-plant objective. It includes two command-memory states and two error integrators. The selected result is `oracle-selected.json`; the initial unscreened design is separately retained as `oracle` in nonlinear artifacts. The screened design is named `oracle_selected`. Neither is an optimal bound on what any possible full-state controller could achieve.

Nonlinear LQI uses truth states, conditional integral updates under saturation, the physical brake actuator and a total-throttle slew limit. Existing-loop slew limiting applies to P+feedforward instead. Thus the controllers share plant and actuator bounds but do not implement identical anti-windup/slew architectures. Wind is not supplied to the LQI as a measured feedforward input. No observer is implemented or tested.

Observability uses a four-second finite horizon. Assumed measurement scales: payload roll/pitch 1 deg, body gyro 1 deg/s, NED navigation velocity 0.1 m/s and height 0.2 m. A hypothetical canopy gyro also uses 1 deg/s. The GNSS velocity Jacobian includes the attitude dependence of the body-coordinate velocity states. These weights are not a calibrated covariance model. The constant-wind augmentation adds three states; neither analysis adds unknown force/moment biases, sensor bias dynamics, estimator delays or correlated AHRS errors. Observability of that nominal model does not resolve the historical unknown-bias ambiguity.

The finite-time amplification calculation starts with physical-state perturbations and zero controller perturbations. Its norm is engineering-scaled, not an energy norm. Its large fast peak can represent displacement-to-rate exchange; it must not be described as tenfold energy amplification or a prediction at a finite 10-degree displacement.

## Retained evidence

- `level.npz`, `climb.npz`, `descent.npz`, `low_voltage.npz`, `dynamic_roll.npz`, `dynamic_climb.npz`: regenerated sampled A/B/E/C, trim state, commands and rotor speed.
- `frequency.json`: independently assembled feedback versus closed-state agreement, and sampled sensitivity peaks. The 500-point frequency sweep is not an exact H-infinity norm computation.
- `state-diagnostics.json`: four-second Gramians, modal participation, input coupling and sensor visibility, including constant-wind augmentation.
- `robustness.json` / `robustness-summary.json`: 648 combinations per conventional tune, including both held-out and training plants. Independent actuator effectiveness [.75, 1.25], independent command delays [0, 40, 80] ms, rotor local rate [.7, 1, 1.3]. These are probes, not measured uncertainty bounds. No disk-margin or structured-singular-value certificate is claimed.
- `oracle-screen.json`, `oracle-selected.json`, `benchmark.json`: declared LQI family and reference-design comparison.
- `nonlinear-summary.json`: matched diagnostic cases. Original four controllers give 52 runs; the screened LQI adds 13. Full 120-second traces are in compressed NPZ files under the ignored `traces/` directory, or the `MICROAGU_STUDY_TRACES` environment-variable path. They are generated by the scripts and excluded from Git, following the repository's raw-data convention. Synthetic gusts end abruptly at 80 seconds; gust metrics exclude that switch.
- `linear-validation.json`: 1 mm ramp equivalence checks, including the selected LQI. Smooth feedforward branch for the conventional controllers in this check only.
- `article-tables.json`: explicit metric windows for the narrative tables.
- `native-baseline.json`, `native-candidate.json`, `native-results.json`, `native-provenance.json`: isolated native parameter overrides, test results and firmware hash. Raw native recordings are outside the repository at the paths recorded in the results.

## Reproduction

Use the locked research uv environment, which builds the Rust Python wheel. Set `OPENBLAS_NUM_THREADS=1` for these small matrices. From the Micro-AGU root:

```sh
export OPENBLAS_NUM_THREADS=1
uv sync --project research --locked --group sitl
PY=research/.venv/bin/python
MODEL=simulation
STUDY=research/archive/state-space-2026-10-14

# Re-solve from retained trim seeds; --cache DIR can instead use historical trims.
$PY $STUDY/study.py prepare --model "$MODEL"
$PY $STUDY/study.py optimize
$PY $STUDY/study.py optimize --constrained
$PY $STUDY/analyze.py
$PY $STUDY/oracle_screen.py
$PY $STUDY/benchmark.py
$PY $STUDY/run_nonlinear.py --model "$MODEL"
$PY $STUDY/report.py
$PY $STUDY/verify.py

# Optional: reproduce the initial unscreened LQI exploratory runs as well.
$PY $STUDY/nonlinear.py --model "$MODEL" --controller oracle
$PY $STUDY/report.py

# Native runs need the existing built SITL executable and a fresh output directory.
$PY $STUDY/native.py --repo /home/david/repos/ardupilot-high-dof \
  --raw /home/david/repos/temp/microagu-state-space-reproduction
$PY $STUDY/native_report.py
$PY $STUDY/audit_native_build.py --repo /home/david/repos/ardupilot-high-dof
$PY $STUDY/verify.py
```

`run_nonlinear.py` runs independent numerical processes, one per controller, then merges results. For a single controller use `nonlinear.py --model "$MODEL" --controller candidate`. Native AUTO runs must remain sequential because the existing harness uses fixed ports and shared SITL log paths. The native fixture starts airborne; launch behavior is not tested. Controller settings are supplied through override files, never written into production defaults.

The backend's legacy `gains: original` label describes the harness preset before overrides. The BIN parameter audits in `verification.json` and `native-results.json` establish which gains actually ran.

The prebuilt native executable reports `ArduPlane V4.8.0-dev (51c8e130)` in its BIN `VER` record. This is distinct from the current checkout `c44434dee6`; we do not claim that the checkout hash identifies the executable's exact build tree. The inspected intervening controller changes concern flare allocation and landing suppression, inactive in these up-and-away AUTO missions. `native-build-audit.json` records each run's version, the binary SHA-256 and the source-difference summary. Rebuilding a later checkout is a new native replication, not a byte-for-byte reproduction of this executable.
