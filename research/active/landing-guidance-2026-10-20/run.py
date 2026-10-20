"""Explicit case manifests; no release artifact changes."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
parser = argparse.ArgumentParser()
parser.add_argument("--campaign", action="store_true")
args = parser.parse_args()
cases = []
conditions = [("calm", {})]
if args.campaign:
    conditions = [("head", dict(along=-1.5, turbulence=.2)),
                  ("cross", dict(across=1.5, turbulence=.2)),
                  ("bias-high", dict(heightBias=.25)),
                  ("bias-low", dict(heightBias=-.25))]
for label, changes in conditions:
    for variant in (["feedforward", "feedback"] if label.startswith("bias") else
                    ["stock", "feedforward", "feedback"]):
        cases.append(dict(id=f"{label}-{variant}", variant=variant, flareHeight=1.05, **changes))
tag = "campaign" if args.campaign else "pilot"
for d in ("cases", "runs"):
    (HERE/d).mkdir(exist_ok=True)
hashes = {}
for path in ["viewer/generated/ardupilot/arduplane.wasm", "viewer/generated/physics/sim_wasm_bg.wasm",
             "aircraft/microagu/controllers/lqi-gains.bin", "aircraft/microagu/configurations/browser-physics.json"]:
    hashes[path] = hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
(HERE/f"{tag}-protocol.json").write_text(json.dumps(dict(cases=cases, artifacts=hashes,
    source={p:hashlib.sha256((HERE/p).read_bytes()).hexdigest() for p in ["case.mts","guidance.ts"]}), indent=2)+"\n")
env = dict(os.environ, PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
execution = []
for case in cases:
    path = HERE/"cases"/(case["id"]+".json")
    path.write_text(json.dumps(case))
    r = subprocess.run([str(ROOT/"viewer/node_modules/.bin/tsx"), str(HERE/"case.mts"),
                        str(path), str(HERE/"runs"/(case["id"]+".json"))],
                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=390)
    execution.append(dict(id=case["id"], code=r.returncode, stdout=r.stdout, stderr=r.stderr))
    (HERE/f"{tag}-execution.json").write_text(json.dumps(execution, indent=2)+"\n")
    print(case["id"], r.returncode, r.stdout[-500:], r.stderr[-500:], flush=True)
