# L1 regression archive

Read [report.md](report.md) for the selection, metrics and plots.

Raw logs and full truth recordings stay in Mission Planner. The scripts default
to the original WSL data directory. Set PG_RESEARCH_ROOT to another raw-data
directory if needed; do not write full telemetry into this Git archive.
Use the ArduPilot Python environment (pymavlink, NumPy, SciPy, Matplotlib).

The existing 8-second mission and landing baselines are referenced from adjacent
Mission Planner studies. The JSON plan records the fresh cases. The run scripts
use the existing native harness in /home/david/repos/ardupilot-high-dof.
The archived scripts differ from their executed raw-directory copies only in
their default output-root selection, so rerunning them keeps raw data outside Git.

The JSON results cache completed audits. Remove the derived mission-results.json
or landing-results.json in the raw directory before recomputing every audit
after changing an analysis formula. Preserve the raw recordings.

No raw BIN, telemetry CSV, NPZ or full viewer recording is included here.

## Landing article and replay update

The landing article uses the selected 10 s L1 profile. A separate native `gentle10` illustration was run after the 42-run regression study, paired with the existing `calm10` run. Both use damping 1.0 and the same 1 Hz inner controller. These preview runs are excluded from the twelve-case statistics.

`research/archive/legacy-tools/report_landing_guidance.py` audits logged parameters and native flare events, generates the publication figures and final regression summary, and exports bounded 22-second truth replays. Run it from the ArduPilot Python environment with `--root` pointing to the Mission Planner study directory. `--run-preview` can generate the gentle preview when that completed recording does not already exist. Raw telemetry remains in Mission Planner.

The blog assets are in `docs/assets/landing-guidance`; the applet excerpts are in `viewer/assets/recordings/landing`.
