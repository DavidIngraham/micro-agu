"""Seeded paired sensitivity study. Run from any directory; each case uses real WASM SITL."""
import argparse
import concurrent.futures
import json
from pathlib import Path
import random
import subprocess

HERE = Path(__file__).resolve().parent
VIEWER = HERE.parents[1] / "viewer"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--pairs", type=int, default=16)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--speedup", type=float, default=8)
    args = parser.parse_args()
    rng = random.Random(20261016)
    (HERE / "cases").mkdir(exist_ok=True)
    (HERE / "results").mkdir(exist_ok=True)
    cases = []
    for i in range(args.pairs):
        # Two nominal controls; remaining draws are assumed engineering sensitivity ranges.
        nominal = i < 2
        spec = dict(id=i, mode="AUTO" if i % 2 == 0 else "CRUISE", speedup=args.speedup,
                    wind=1 if nominal else rng.uniform(0, 2),
                    **{"from": 90 if nominal else rng.uniform(0, 360)},
                    weather=dict(turbulence=0.2 if nominal else rng.uniform(0, 0.6),
                                 thermalStrength=1.5 if nominal else rng.uniform(0, 2),
                                 thermalRadius=60, thermalNorth=300, thermalEast=0,
                                 seed=rng.randrange(1, 2**32)),
                    scales={} if nominal else {
                        key: rng.uniform(0.7, 1.3) for key in
                        ["Cmq", "joint_c_pitch", "joint_c_roll", "joint_c_yaw", "CLbrake", "CDbrake"]},
                    brakeRateScale=1 if nominal else rng.uniform(0.7, 1.1))
        path = HERE / "cases" / f"{i:02}.json"
        path.write_text(json.dumps(spec, indent=2) + "\n")
        cases.extend((path, scheme) for scheme in ("baseline", "lqi"))

    def run(job):
        path, scheme = job
        output = HERE / "results" / f"{path.stem}-{scheme}.json"
        output.unlink(missing_ok=True)  # A failed rerun must not reuse a prior success.
        with output.with_suffix(".log").open("w") as log:
            try:
                result = subprocess.run([str(VIEWER / "node_modules/.bin/tsx"),
                                         "tests/integration/stability-case.ts", str(path), scheme, str(output)],
                                        cwd=VIEWER, stdout=log, stderr=subprocess.STDOUT, timeout=330)
                code = result.returncode
            except subprocess.TimeoutExpired:
                code = 124
            if not output.exists():
                output.write_text(json.dumps(dict(spec=json.loads(path.read_text()), scheme=scheme,
                                                 complete=False, failure=f"Runner failure, exit {code}",
                                                 samples=[])) + "\n")
        return path.stem, scheme, code

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        for result in pool.map(run, cases):
            print(result, flush=True)


if __name__ == "__main__":
    main()
