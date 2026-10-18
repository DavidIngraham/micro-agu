"""Capture inputs and tool versions alongside retained experimental results."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
paths = ["viewer/generated/ardupilot/arduplane.wasm",
         "viewer/generated/ardupilot/arduplane.js",
         "viewer/generated/physics/sim_wasm_bg.wasm",
         "viewer/assets/runtime/highdof.json",
         "viewer/tests/integration/stability-case.ts",
         "viewer/packages/sim-browser/src/session/runtime.ts",
         "viewer/packages/sim-browser/src/physics/atmosphere.ts",
         "research/archive/controller-stability-2026-10-16/run.py",
         "research/archive/controller-stability-2026-10-16/analyze.py"]
report = dict(git_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              note="Harness changes are uncommitted; exact source hashes below. No firmware retune.",
              python=sys.version,
              sha256={p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths})
(HERE / "provenance.json").write_text(json.dumps(report, indent=2) + "\n")
