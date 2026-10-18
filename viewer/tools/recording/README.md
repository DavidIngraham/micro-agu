# Recording and live bridge tools

Reusable local UDP/WebSocket acquisition tools promoted from the viewer experiments. Use the research environment's `capture` group, which supplies the simulator and WebSocket dependencies:

```sh
uv run --project research --group capture python viewer/tools/recording/bridge.py --help
uv run --project research --group capture python -m unittest discover -s viewer/tools/recording -p 'test_*.py'
```

Study-specific analysis and scenario generators remain in `research/archive/viewer-studies`. These tools do not belong to the browser runtime bundle.
