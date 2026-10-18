# Repository organization

MicroAGU remains one repository. Package boundaries allow later extraction without maintaining multiple repositories today.

| Owner | Maintained responsibility |
|---|---|
| `simulation/` | Rust physics, I/O schemas, native/Python/WASM bindings and numerical tests |
| `control/` | Offline identification, estimation, synthesis, campaign assessment and gain artifacts |
| `viewer/` | Shared rendering, applets, browser simulator and recording tools |
| `aircraft/microagu/` | Selected measured inputs, production configuration, gains and validation |
| `research/active/` | Current model-fidelity and pipeline studies |
| `research/archive/` | Historical evidence and superseded methods |
| `docs/`, `cad/` | Project articles and mechanical design |
| `tools/`, `provenance/` | Repository maintenance and recoverability records |

Dependencies flow from control and applications into public simulator interfaces. The simulator must not import the control package or research code. Runtime builds consume selected configuration; they never run an experiment or optimize gains.

## Recoverability

The pre-cleanup Git tag is `archive/pre-layout-2026-10-18`. It preserves original tracked paths, including the old scratch archive. A separate checksummed working-tree snapshot captures local modifications, untracked experiments and ignored research outputs. Its local path and hash are recorded in [the migration manifest](../provenance/layout-migration-2026-10-18.json). This backup is local, not a remotely hosted artifact.

Frozen experimental JSON, gain files and recorded source hashes remain evidence of the original run. Their historical absolute paths are not silently rewritten. Source and documentation links have moved; the migration manifest maps old directories to their replacements.

## Routine checks

```sh
python3 tools/check_repository.py
uv sync --project control --locked
uv run --project control pytest control/tests -q
uv sync --project simulation --locked
uv run --project simulation pytest simulation/tests -q
cd viewer && npm test && npm run build
```

The retired root Python import shims are removed. Analysis callers import `microagu_sim.compat` explicitly; identification and synthesis callers import `microagu_control`. New reusable work should use public APIs and installed packages rather than modifying `sys.path` or copying a study runner.

No Git history rewrite or simulator redeployment is part of this layout change.

The October 18 cleanup reduced the tracked tree from 4,281 files to 2,834 while preserving previously untracked useful study evidence. It removed 14 Python root shims and consolidated 958 nested case artifacts into eight verified evidence archives. See [validation results](../provenance/layout-validation-2026-10-18.json).
