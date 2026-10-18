# Aircraft identification and control tools

This independently installable Python package owns offline identification, hidden-state estimation, gain synthesis, campaign assessment and gain-file export. The Rust physics and its bindings remain in `../simulation`; the viewer remains in `../viewer`.

```sh
cd control
uv sync --locked
uv run pytest -q
uv run microagu-sysid --help
uv run microagu-tune --help
```

The equivalent module entry points are `python -m microagu_control.sysid` and `python -m microagu_control.pipeline`. The former `microagu_sim.sysid` and `microagu_sim.pipeline` namespaces have been retired; update callers rather than installing compatibility wrappers.

- [Identification contracts and prepared data](src/microagu_control/sysid/README.md)
- [Pipeline diagram, measured inputs and gain-file contract](src/microagu_control/pipeline/README.md)
- [Current aircraft and deployed configuration](../aircraft/microagu/README.md)
- [Active research](../research/README.md)

Dependencies flow from control to the public simulator API. Neither package imports a research study or the web application. `microagu_sim.compat` remains an explicit simulator adapter for the existing analysis coordinate conventions; its internals have not been redesigned during this move.

`estimation.py` retains the offline conditional MAP estimator. It is a research analysis capability, not an onboard observer or AHRS implementation. Controller decoder tests additionally use `MICROAGU_ARDUPILOT=/path/to/ardupilot`; without that checkout those tests explicitly skip.

New experimental fitting methods belong in an active study until they have reusable interfaces and tests. Outputs go to a new run directory; selected releases are promoted explicitly into `aircraft/`.
