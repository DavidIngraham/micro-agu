"""Run existing campaign cases without changing controller gains or sensor settings."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
VIEWER = ROOT / 'viewer'


def main():
    out = HERE / sys.argv[1]
    ids = [int(i) for i in sys.argv[2].split(',')] if len(sys.argv) > 2 else list(range(16))
    (out / 'results').mkdir(parents=True, exist_ok=True)
    shutil.copyfile(VIEWER / 'assets/runtime/build.json', out / 'build.json')
    shutil.copyfile(VIEWER / 'tests/integration/stability-case.ts', out / 'stability-case.ts.txt')

    def run(i):
        output = out / 'results' / f'{i:02}-lqi.json'
        assert not output.exists(), output
        case = HERE.parent / 'controller-stability-2026-10-16/cases' / f'{i:02}.json'
        with output.with_suffix('.log').open('w') as log:
            p = subprocess.run([str(VIEWER / 'node_modules/.bin/tsx'), 'tests/integration/stability-case.ts',
                                str(case), 'lqi', str(output)], cwd=VIEWER,
                               stdout=log, stderr=subprocess.STDOUT, timeout=330)
        print(i, p.returncode, flush=True)
        return p.returncode

    with ThreadPoolExecutor(max_workers=8) as pool:
        codes = list(pool.map(run, ids))
    if any(codes):
        raise SystemExit('One or more cases failed; inspect retained logs.')


if __name__ == '__main__':
    main()
