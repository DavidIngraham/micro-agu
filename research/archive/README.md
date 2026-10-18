# Archived studies

These directories retain evidence and source context, not supported command-line applications. The historical checkout at tag `archive/pre-layout-2026-10-18` preserves original paths. Ongoing work lives in [active](../active/).

| Study | Disposition |
|---|---|
| [airspeed-sensor-2026-10-16](airspeed-sensor-2026-10-16/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [as-built-simulator-2026-10-15](as-built-simulator-2026-10-15/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [brake-routing-2026-10-15](brake-routing-2026-10-15/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [controller-stability-2026-10-16](controller-stability-2026-10-16/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [filtered-brake-2026-10-16](filtered-brake-2026-10-16/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [flare-authority-2026-10-13](flare-authority-2026-10-13/) | deferred: Landing/envelope exploration outside the current flight-identification focus. |
| [flight-log-analysis](flight-log-analysis/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [hidden-state-map-2026-10-16](hidden-state-map-2026-10-16/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [low-speed-ekf-2026-10-16](low-speed-ekf-2026-10-16/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [lqi-backport-2026-10-17](lqi-backport-2026-10-17/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [lqi-balanced-2026-10-17](lqi-balanced-2026-10-17/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [lqi-feasible-turns-2026-10-17](lqi-feasible-turns-2026-10-17/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [lqi-filter-retune-2026-10-17](lqi-filter-retune-2026-10-17/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [lqi-longitudinal-2026-10-17](lqi-longitudinal-2026-10-17/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [lqi-transitions-2026-10-17](lqi-transitions-2026-10-17/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [lqi-turn-envelope-2026-10-17](lqi-turn-envelope-2026-10-17/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [native-mimo-2026-10-16](native-mimo-2026-10-16/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [proprioception-2026-10-17](proprioception-2026-10-17/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [rigging-refinement-2026-10-15](rigging-refinement-2026-10-15/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [scheduled-state-feedback-2026-10-15](scheduled-state-feedback-2026-10-15/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [sitl-studies](sitl-studies/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [spiral-authority-2026-10-13](spiral-authority-2026-10-13/) | deferred: Landing/envelope exploration outside the current flight-identification focus. |
| [spiral-swoop-2026-10-13](spiral-swoop-2026-10-13/) | deferred: Landing/envelope exploration outside the current flight-identification focus. |
| [state-space-2026-10-14](state-space-2026-10-14/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |
| [swoop-2026-10-13](swoop-2026-10-13/) | deferred: Landing/envelope exploration outside the current flight-identification focus. |
| [tension-stiffness-2026-10-15](tension-stiffness-2026-10-15/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [viewer-studies](viewer-studies/) | reference: Retained measurements, published figures, model lineage or current-envelope evidence. |
| [wind-calibration-2026-10-16](wind-calibration-2026-10-16/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [wind-smoother-2026-10-16](wind-smoother-2026-10-16/) | completed-unresolved: Completed experiment with conclusions still relevant to sensing or model uncertainty. |
| [zero-brake-2026-10-16](zero-brake-2026-10-16/) | superseded: Older controller/design iteration; current selected configuration is aircraft/microagu/current.json. |

`legacy-tools/` holds the former root analysis/report scripts. `legacy-control/` retains the standalone pre-pipeline gain designer and its experimental tests, including pre-cleanup local edits. They are not the source of the deployed gain file.

The old `scratch/` implementation snapshots are no longer in this working tree. Recover tracked content from the pre-layout tag. `tools/restore_artifacts.py` can still verify and restore the original compressed archive using that tag. The checksummed local working-tree backup also preserves formerly untracked work; its location is recorded in [the migration manifest](../../provenance/layout-migration-2026-10-18.json).

Large collections of nested tuning cases are consolidated into per-study `evidence.tar.gz` archives. Each has a manifest with per-file checksums and an `ARCHIVE.md` describing restoration. Use `tools/restore_study.py`; reports and directly linked artifacts stay browsable.
