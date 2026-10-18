# Migrated SITL and controller studies

These are experiment drivers and historical records, separate from the reusable
[simulator](../../../simulation/README.md). They originated in the ArduPilot external
model directory; [the migration manifest](../../../provenance/simulator-migration.json)
preserves original hashes. Files moved with Git retain their history.

| Files | Purpose |
| --- | --- |
| `compare_autotest.py`, `damper_autotest.py`, `longitudinal_autotest.py` | Native firmware comparison and ablation experiments |
| `landing_*` | Landing cases, Monte Carlo experiments and reports |
| `controllers.py`, `mission_demo.py` | Standalone diagnostic controllers and mission comparisons |
| `*_closed_joint_tour.py` | Native AUTO joint-release experiment and viewer-result packaging |
| `study-*`, `study_*.py`, `longitudinal_frequency.py` | Study configurations, trim checks and frequency analysis |
| `analyze_*`, `plot_*`, `report_*`, `compare_longitudinal_study.py` | Analysis and report generation |
| Historical Markdown, text and provenance JSON | Original methods, assumptions and results |

Studies import the installed `microagu_sim.compat` package. `_sim_path.py`
locates repository parameter files only; it does not modify Python's import path.
Select another simulator checkout with `uv`'s path dependency, then resync.
`MICROAGU_SIM_ROOT` selects the associated firmware parameter files for recordings.

```sh
cd research
uv sync --locked --group sitl
export ARDUPILOT_ROOT=/path/to/ardupilot
uv run --group sitl python sitl-studies/compare_autotest.py --help
uv run python sitl-studies/mission_demo.py --help
```

The ArduPilot checkout needs its normal autotest dependencies and a built SITL
binary for native runs. Save raw outputs in a separate run directory. Historical
notes preserve their original paths and commands; use the paths above for the
current layout. Do not interpret old snapshot notes as current build instructions.
