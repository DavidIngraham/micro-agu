"""Bounded sensitivity runs using the existing packaged-runtime landing driver."""
import concurrent.futures
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def run(spec):
    folder = HERE / "runs"
    folder.mkdir(exist_ok=True)
    src = folder / (spec["id"] + ".json")
    src.write_text(json.dumps(spec))
    env = dict(os.environ, PATH="/home/david/emsdk/node/24.19.0_64bit/bin:/usr/bin:/bin")
    with (folder / (spec["id"] + ".log")).open("w") as log:
        result = subprocess.run(
            [str(ROOT / "viewer/node_modules/.bin/tsx"), str(HERE / "case.mts"),
             str(src), str(folder / (spec["id"] + "-result.json"))],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=400,
        )
    print(spec["id"], result.returncode, flush=True)
    return result.returncode


if __name__ == "__main__":
    specs = json.loads((HERE / "cases.json").read_text(encoding="utf-8-sig"))
    if len(sys.argv) > 1:
        specs = [s for s in specs if s["id"] in sys.argv[1:]]
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run, specs))
    raise SystemExit(any(results))
