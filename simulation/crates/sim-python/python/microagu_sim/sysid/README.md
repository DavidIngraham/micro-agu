# Offline grey-box identification

This package provides the reusable analysis foundation for the flight-test workbench. It runs the existing as-built Rust paraglider physics via the Python bindings; it does not install models, write aircraft parameters, or generate controller gains.

## Run

Use the simulation environment with its `analysis` extra:

```sh
simulation/.venv/bin/python -m microagu_sim.sysid fit session.json runs/fit-001
simulation/.venv/bin/python -m microagu_sim.sysid validate session.json runs/check-001 --fit-result runs/fit-001/fit.json
simulation/.venv/bin/pytest -q simulation/tests/test_sysid.py
```

Output directories must be new. The CLI writes an input/source manifest before the job, and the result after successful completion. An interrupted run can leave its manifest without a result. There is no automatic deployment or acceptance decision. Fit results include optimizer status, replay counts, candidate trace, and training-only selection. Validation is free-running on reserved episodes; no parameters or nuisance states are optimized using evaluation outputs.

## Session schema

A version-1 JSON session contains `segments`, optional `fit` options and optional `starts`. Each segment declares `name`, `episode`, `role` (`train` or `validation`), `flight` (`trout_lake` or `hood_river`), `start_s`, `duration_s`, `causal_input`, and `propulsion_input`. Paths resolve relative to the session file. These initial adapters use existing as-built profiles; arbitrary-aircraft configuration is future work.

Prepared input contract:

- Causal NPZ: `t` in aircraft seconds, `Y` with Euler XYZ radians/body rates rad/s/IMU-point NED velocity m/s, `U` with throttle in column zero, `brake_mm` with left/right commanded pull, `valid`, `wind` in NED m/s, and `force` as earth-frame specific force m/s².
- Propulsion NPZ: matching `t`, `voltage`, and `rpm`.
- Uniform 25 Hz samples, at least 12.8 s per segment. Gaps, nonfinite values and mismatched clocks are rejected. This first version consumes prepared causal logs, not raw DataFlash files.

Training/validation episode IDs must be disjoint; cross-role time overlap and duplicate names are rejected. Episode labels are supplied by the caller and must represent complete maneuvers honestly. Repeated initial-angle evaluations are sensitivity cases, not independent observations.

Optional `ground_wind` contains `path`, explicit `time_offset_s` mapping CSV time into aircraft time, `airborne_difference_sigma_m_s`, and optional `max_gap_s` (default 3). CSV columns are `time_s,north_m_s,east_m_s`, in earth-frame **toward** components. Convert meteorological **from** direction and align sensor orientation before import. The importer requires coverage and rejects long gaps, then uses a mean per segment. It does not inject ground gust samples as aircraft truth. The uncertainty represents ground/air differences, not just anemometer accuracy. Without CSV input, existing prepared wind is used with a default 2 m/s prior scale; its historical origin must be retained in session provenance.

## Estimation choices

`FitOptions` supports `objective` (`response`, default, or `legacy`), `fit_nuisance` (false by default), `max_replays` (2,000 per start), `workers`, `substeps`, `regularization`, `nuisance_weight`, and `fd_step`. Parameters are normalized by their bounds. Finite-difference probes count against the budget, and unaffected windows are cached across nuisance probes. The cache is bounded. Failed numerical replays raise an error instead of fabricating a trajectory.

The six fitted quantities are rotor inertia, yaw stiffness, yaw damping, pitch damping, effective brake pull and effective first-order command lag. All other aircraft quantities remain fixed in the chosen profile. Bounds are exploratory, not measurement confidence intervals. The legacy objective reproduces the previous rate/acceleration, spectral-amplitude and pitch/yaw phase combination. The response objective fits uncentered rates and accelerations; independent validation still reports amplitude, inter-axis phase, command-response phase and residual correlation.

Optional nuisance fitting permits a bounded initial canopy-angle correction and constant north/east wind corrections for each training segment. These are penalized, not unconstrained motion corrections. There is no dynamic latent-force or stochastic innovations model yet. Nuisance corrections are not applied to validation windows.

`diagnostics.sensitivity` exposes inactive parameters, singular values, conditional sensitivity and bound hits. It is not a covariance estimator; it does not marginalize nuisance variables or account for colored noise and feedback. Short-window cross-spectral coherence is descriptive and has no white-noise significance claim.

## Evidence and limitations

See `research/greybox-pipeline-2026-10-18` for exact baseline reproduction, frozen real-data comparisons, known-physics synthetic recovery, integration-resolution audits and CLI smoke tests. The noise-free synthetic case verifies six-parameter recovery, but uses the same simulator for generation and fitting. The real-data candidates still miss oscillation energy and reach parameter bounds. None is promoted to production or used for LQI retuning.

### Measured aircraft profile

An optional session `aircraft_profile` path loads the versioned profile produced by `python -m microagu_sim.pipeline init aircraft.json`. It is applied during every identification and validation replay, and its hash is bound into CLI fit outputs. The current adapter supports the Trout Lake as-built geometry. Validation rejects a different profile from the fitted one. Omitting the field retains the historical profile exactly.
