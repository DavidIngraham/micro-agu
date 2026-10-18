# Feasible turn trims with unchanged LQI gains

AI-assisted research implementation and SITL experiment, 2026-10-17.

## Outcome

35 deg/s follow-up: Pages commit `65c0cb9` raises the browser limit to 35 after 14 nominal/transition flights and 18 fresh paired disturbance flights pass the frozen release criteria. Gains, observer, trim map and engagement blend are unchanged. Local/live browser checks and all 25 public asset hashes pass. See [35 deg/s results](deployment35/RESULTS.md). The shared `PG_TURN_RMAX` also becomes 35 so that rate is reachable; PG + TECS was tested at 35 and remains the default. Firmware SHA-256: `4995915df52e4977185e2f7e1e4616118576a9883923a7398679edcf6876a49e`. Earlier results below are retained as history.

Final follow-up: the browser-only 25 deg/s candidate was published in Pages commit `7db7525`. A two-second engagement blend fixes the identified FBWA-to-LQI transient. All 16 final-binary qualification flights passed, including fresh transitions; local and public browser smoke tests passed, and all 25 public asset hashes matched. Firmware SHA-256: `7288fcc2de2da91965ce6ea9eaf66be520859042f9be9a7906ace2a090793238`. See `deployment-v2/qualification.json`, `deployment-v2/fresh/qualification.json`, `deployment-v2/audit.json`, and the packaged [release notes](../../../viewer/LQI-FEASIBLE-TURNS.md). The original disturbance screens and initial handover failure are preserved below; the 40 deg/s envelope is not deployed.

Follow-up release validation: **not accepted; no push or deployment**. All 18 fresh flights completed and passed quiet recovery, but none passed every original disturbance gate. See [release validation](RELEASE-VALIDATION.md). Transition qualification is deferred after this prerequisite failure. The follow-up adds opt-in weather removal to the harness; prior runs keep their original continuous-weather behavior.

The combined feasible-reference map, actuator-specific anti-windup, and observer equilibrium recentering passed both nominal direction sequences at 10, 15, 20, 25, 30, 35 and 40 deg/s. Feedback K and observer L are identical to the published controller. Gain scheduling is not justified by this nominal sweep alone.

At a 25 deg/s command, the reference averaged 31.59 right / -25.68 left; the combined candidate averaged 25.44 / -25.30. At 40 deg/s the combined candidate averaged 40.38 / -40.30, with each flight's peak altitude error below 0.5 m. Anti-windup alone passed through 35 but failed at 40: one flight crossed the attitude guard, and the other failed tracking. Local pole checks did not predict that nonlinear failure.

In six fresh disturbed cases per variant at 25 deg/s, all 24 flights completed, but none passed every original motion/recovery screen. Weather remained active during recovery: these scores include forced motion, not just free decay. Do not interpret completion as a stability proof, or discard the failed screens. Combined-candidate average worst-phase mean tracking error was 0.91 deg/s versus 6.60 for the reference; modeled shaft power averaged 77.21 W versus 79.70 W (3.1% lower). Late relative pitch-rate RMS was slightly worse, 5.14 versus 5.07 deg/s. This is primarily a tracking improvement, not evidence of uniformly better disturbance rejection.

Nominal observer canopy pitch-rate RMS error at 25 deg/s fell from approximately 7.4–7.6 to 1.0–1.1 deg/s with equilibrium recentering. Other components did not uniformly improve, including canopy yaw rate. Truth was used for scoring only; the controller retained EKF airspeed and measured heading rate, without pitot or canopy truth inputs.

## Implementation

- `trim_grid.py`: constrained nonlinear equilibria using the as-built Rust model; 85 attempted nodes across signed turn rate and airspeed. The table accepts 55 converged nodes through +/-40 deg/s with residual below 1e-6. A failed solve is not proof of global infeasibility.
- `modify.py`: reproducible controller transformation. `CP3_AWMODE=1` prevents clipped common brake from cancelling turn-integrator correction. `CP3_TMAP=1` schedules equilibrium references/feedforward and governs speed to the tabulated branch. `CP3_OBSTRIM=1` recenters the fixed observer prediction about an estimated operating equilibrium. All new switches default off.
- `build.py`: isolated WASM build with restoration of all 21 original source/runtime files. `runtime/production-LQI.cpp` omits research telemetry; `runtime/source.patch` includes instrumentation. These are research artifacts, not an applied production migration.
- `case.mts`, `run.py`, campaign scripts and frozen protocol JSON files: real AP-in-the-loop flights, reversals and recovery; three-way parallel execution per batch.
- `local_analysis.py`: local linear screening at exact equilibria. Scheduling derivatives, actuator saturation and EKF dynamics are not covered. Some 60 deg/s nodes are locally unstable with fixed gains; this does not establish the usable nonlinear boundary.
- `score.py`, `report.py`, `assess_observer.py`, `audit.py`: unchanged absolute screens, comparative summaries, observer error and provenance/restoration checks.

## Limits and next decision

Use this research runtime only with `CP3_RMAX <= 40` when the trim map is enabled. The table clamps its lookup at 40, while the general experimental rate parameter still permits 60. A production integration must constrain the command to table coverage before raising the public limit. The nominal extension stops at the table boundary, not a demonstrated physical maximum.

Keep the public simulator unchanged pending broader uncertain-model tests at the expanded rates, weather-off recovery tests that separate forced motion from damping, and engagement/disengagement checks. Preserve the existing disturbed screens alongside any new recovery protocol. Add gain scheduling only if these tests identify a repeatable dynamics mismatch that equilibrium scheduling and actuator handling cannot address.

## Evidence and reproduction

See [all flight results](RESULTS.md), `summary.json`, `results.json`, `observer-errors.json`, `local-poles.json`, and `audit.json`. Per-case inputs, logs and traces remain under `ablation/`, `extension/` and `validation/`. Cohort summary columns are averages across cases, including columns named peak; consult individual cases for worst-case limits.

Run trim generation with `simulation/.venv/bin/python trim_grid.py`. Build and analysis scripts use the existing `/home/david/envs/venv-ardupilot/bin/python` environment. Run `build.py` only with no flight jobs active, followed by `campaign.py`, `extend.py`, `extend_map.py`, and `validate.py`; then `report.py`, `assess_observer.py`, `local_analysis.py`, and `audit.py`. Existing results are reused by the batch driver. Paths and toolchain dependencies are encoded in the research scripts; this is not yet a portable release pipeline.
