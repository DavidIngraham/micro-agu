"""Run the unchanged scenarios with a calibrated, noisy SITL airspeed sensor."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
VIEWER = ROOT / 'viewer'
PARAMETERS = dict(ARSPD_TYPE=100, ARSPD_USE=1, ARSPD_SKIP_CAL=1, ARSPD_OFFSET=0,
                  ARSPD_RATIO=1.99, SIM_ARSPD_RATIO=1.99, SIM_ARSPD_RND=2)


def main():
    name = sys.argv[1]
    ids = [int(i) for i in sys.argv[2].split(',')] if len(sys.argv) > 2 else list(range(16))
    out = HERE / name
    (out / 'results').mkdir(parents=True, exist_ok=True)
    (HERE / 'cases').mkdir(exist_ok=True)
    shutil.copyfile(VIEWER / 'assets/runtime/build.json', out / 'build.json')
    shutil.copyfile(VIEWER / 'tests/integration/stability-case.ts', out / 'stability-case.ts.txt')
    for i in ids:
        spec = json.loads((HERE.parent / 'controller-stability-2026-10-16/cases' / f'{i:02}.json').read_text())
        spec['parameters'] = PARAMETERS
        (HERE / 'cases' / f'{i:02}.json').write_text(json.dumps(spec, indent=2)+'\n')

    def run(i):
        output = out / 'results' / f'{i:02}-lqi.json'
        assert not output.exists(), output
        with output.with_suffix('.log').open('w') as log:
            p = subprocess.run([str(VIEWER / 'node_modules/.bin/tsx'), 'tests/integration/stability-case.ts',
                                str(HERE / 'cases' / f'{i:02}.json'), 'lqi', str(output)],
                               cwd=VIEWER, stdout=log, stderr=subprocess.STDOUT, timeout=330)
        print(i, p.returncode, flush=True)
        return p.returncode

    with ThreadPoolExecutor(max_workers=8) as pool:
        codes = list(pool.map(run, ids))
    if any(codes):
        raise SystemExit('One or more cases failed; retain logs and inspect.')


if __name__ == '__main__':
    main()
