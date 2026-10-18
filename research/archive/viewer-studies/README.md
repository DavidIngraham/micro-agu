# Viewer data production studies

Scenario generators, native truth capture, synthetic gust experiments and analysis
live here. These are offline producers of recordings, not dependencies of the web
viewer or reusable Rust simulation core.

From the repository root:

```sh
uv sync --project research --locked --group capture --group sitl
uv run --project research --group capture python -m unittest discover -s research/archive/viewer-studies -p 'test_*.py'
uv run --project research --group capture python viewer/tools/recording/check_live.py --model-dir simulation
```

Individual CLI scripts retain their arguments and use the installed
`microagu_sim.compat` adapters. Native AUTO capture requires a built ArduPilot
checkout and its normal autotest environment. Keep raw outputs in the existing
flight-analysis storage. Export bounded display excerpts into
`viewer/assets/recordings`, review their provenance, then explicitly update
`viewer/assets/manifests/recordings.json` with `npm run assets:inventory`.

The generators do not run during a viewer build or publication. Detailed invocation
recipes are in [the viewer recipes](../../../viewer/docs/RECIPES.md).
