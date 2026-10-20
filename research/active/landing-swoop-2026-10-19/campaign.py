"""Sequential AP/WASM cases using selected production artifacts, no patches."""
from pathlib import Path
import hashlib
import json
import os
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
cases = []
for scheme in ("baseline", "lqi"):
    for label, north, east, turbulence in (("calm", 0, 0, 0),
                                           ("head", -1.5, 0, .2),
                                           ("cross", 0, 1.5, .2)):
        cases.append(dict(id=f"land-{scheme}-{label}", kind="landing",
                          scheme=scheme, north=north, east=east, turbulence=turbulence))
    for rate in (0, 25):
        cases.append(dict(id=f"recover-{scheme}-{rate}", kind="recovery",
                          scheme=scheme, turnRate=rate))
env = dict(os.environ, PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
for folder in ("cases", "ap-runs"):
    (HERE/folder).mkdir(exist_ok=True)
manifest = dict(cases=cases, artifacts={})
for name in ("viewer/generated/ardupilot/arduplane.wasm",
             "viewer/generated/physics/sim_wasm_bg.wasm",
             "aircraft/microagu/controllers/lqi-gains.bin",
             "aircraft/microagu/configurations/browser-physics.json"):
    p = ROOT/name
    if p.exists():
        manifest["artifacts"][name] = hashlib.sha256(p.read_bytes()).hexdigest()
(HERE/"ap-protocol.json").write_text(json.dumps(manifest, indent=2)+"\n")
results = []
for case in cases:
    path = HERE/"cases"/(case["id"]+".json")
    path.write_text(json.dumps(case))
    result = subprocess.run([str(ROOT/"viewer/node_modules/.bin/tsx"), str(HERE/"case.mts"),
                             str(path), str(HERE/"ap-runs"/(case["id"]+".json"))],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=330)
    results.append(dict(id=case["id"], code=result.returncode,
                        stdout=result.stdout, stderr=result.stderr))
    print(case["id"], result.returncode, result.stdout[-500:], result.stderr[-300:], flush=True)
    (HERE/"ap-execution.json").write_text(json.dumps(results, indent=2)+"\n")
