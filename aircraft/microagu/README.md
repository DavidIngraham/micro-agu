# Current MicroAGU configuration

[`current.json`](current.json) is the single index of the selected aircraft inputs, production physics, LQI gains, firmware revision, validation evidence and published browser release. Its hashes bind the small release bundle in this directory.

- `configurations/measured.json`: measured Trout Lake mass and geometry, with provenance and explicit unknown uncertainties.
- `configurations/browser-physics.json`: the production browser model and operating defaults. The exploratory fitted physics has **not** been promoted.
- `controllers/`: selected feedback/anti-windup file and its generated header. The file requires the matching compiled observer/model context.
- `validation/`: retained acceptance evidence. The present claim is practical simulation non-regression, not demonstrated overall improvement or flight qualification.

The browser's copies in `viewer/assets/runtime/` are staged release artifacts. `python3 tools/check_repository.py` at the repository root verifies them against this bundle, and also checks the firmware hash and declared gain-file hash. A future promotion must update both the bundle and staged assets deliberately. Ordinary builds do not retune or alter aircraft configuration.

Historical input paths inside frozen validation records describe their original runs; use the research catalog and migration manifest to locate moved studies. Do not rewrite those records to pretend experiments ran at different paths.
