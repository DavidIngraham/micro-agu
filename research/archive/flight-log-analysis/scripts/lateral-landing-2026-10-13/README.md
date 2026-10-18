# Landing lateral study

Raw data and generated working outputs remain under Mission Planner:
`analysis/lateral-tuning-2026-10-13`. The scripts here retain that explicit local
path and the `/home/david/repos/ardupilot-high-dof` model checkout.
Run with `/home/david/envs/venv-ardupilot/bin/python`.

- `descent_linear.py`: balanced descent trims, plant Bode response, feedback poles.
- `validate_descent.py`: small nonlinear sine checks of those transfer functions.
- `delay_modes.py`: explicit pure-command-delay sensitivity.
- `analyze.py`: initial matched landing screen, including unfiltered truth rate.
- `run_screen.py`: initial 4-Hz mission-lateral landing comparison.

The maintained publication tools are at repository root:
`research/archive/legacy-tools/report_landing_lateral.py` and `research/archive/legacy-tools/report_lateral_missions.py`.
They create the final figures, parameter/log audits and bounded applet replays.
The opt-in mission profile now selects 1 Hz and the landing harness shares its
lateral gains. No raw BIN, telemetry or full simulation traces are archived here.

`run_validation.py` and `run_mission_validation.py` reproduce the native
landing/mission campaign sequentially (shared SITL ports). The adjacent JSON
files preserve the exact overrides; copy them to the raw output directory
before running on a new machine. Reuse the prior seeded conditions for a
matched comparison; use a new directory to avoid overwriting raw evidence.
