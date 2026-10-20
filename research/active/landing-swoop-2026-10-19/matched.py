"""Match requested altitude ramps without retuning classical TECS feedforward."""
from pathlib import Path
import json
import os
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
cases = [dict(id=f"matched-input-{scheme}-{rate}", kind="recovery", scheme=scheme,
              turnRate=rate, descentLever=.25, climbLever=.6875)
         for scheme in ("baseline", "lqi") for rate in (0, 25)]
(HERE/"matched-input-protocol.json").write_text(json.dumps(cases, indent=2)+"\n")
env = dict(os.environ, PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
results = []
for case in cases:
    p = HERE/"cases"/(case["id"]+".json")
    p.write_text(json.dumps(case))
    result = subprocess.run([str(ROOT/"viewer/node_modules/.bin/tsx"), str(HERE/"case.mts"),
                             str(p), str(HERE/"ap-runs"/(case["id"]+".json"))],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=330)
    results.append(dict(id=case["id"], code=result.returncode,
                        stdout=result.stdout, stderr=result.stderr))
    print(case["id"], result.returncode, result.stdout[-500:], result.stderr[-300:], flush=True)
    (HERE/"matched-input-execution.json").write_text(json.dumps(results, indent=2)+"\n")
