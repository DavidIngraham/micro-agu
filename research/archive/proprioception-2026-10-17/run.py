"""Matched AP/EKF-in-loop sensing campaign; retain failures and provenance."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
VIEW = ROOT/'viewer'
AIR = dict(ARSPD_TYPE=100, ARSPD_USE=1, ARSPD_SKIP_CAL=1, ARSPD_OFFSET=0,
           ARSPD_RATIO=1.99, SIM_ARSPD_RATIO=1.99, SIM_ARSPD_RND=2)
VARIANTS = {'sensorless': ('none', False, 'lqi'), 'airspeed': ('none', True, 'lqi'),
            'angles': ('angles', True, 'lqi'), 'full': ('full', True, 'lqi'),
            'sensor': ('sensor', True, 'lqi'), 'stress': ('stress', True, 'lqi'),
            'standard': ('none', True, 'baseline')}


def main():
    group = sys.argv[1]
    variants = sys.argv[2].split(',')
    ids = [int(x) for x in sys.argv[3].split(',')]
    tasks = []
    for variant in variants:
        out = HERE/group/variant
        (out/'results').mkdir(parents=True, exist_ok=True)
        (out/'cases').mkdir(exist_ok=True)
        shutil.copyfile(VIEW/'assets/runtime/build.json', out/'build.json')
        for i in ids:
            spec = json.loads((HERE.parent/'controller-stability-2026-10-16/cases'/f'{i:02}.json').read_text())
            kind, air, scheme = VARIANTS[variant]
            spec['parameters'] = AIR if air else {'ARSPD_TYPE': 0, 'ARSPD_USE': 0}
            spec['research_sensor'] = variant
            case = out/'cases'/f'{i:02}.json'
            case.write_text(json.dumps(spec, indent=2)+'\n')
            output = out/'results'/f'{i:02}-{scheme}.json'
            if output.exists():
                continue
            tasks.append((case, output, kind, scheme))
    def run(job):
        case, output, kind, scheme = job
        env = dict(os.environ, PROPRIO_CASE=kind)
        with output.with_suffix('.log').open('w') as log:
            try:
                p = subprocess.run([str(VIEW/'node_modules/.bin/tsx'), 'tests/integration/stability-case.ts',
                                    str(case), scheme, str(output)], cwd=VIEW, env=env,
                                   stdout=log, stderr=subprocess.STDOUT, timeout=330)
                code = p.returncode
            except subprocess.TimeoutExpired:
                code = 124
        print(output.relative_to(HERE), code, flush=True)
        return code
    with ThreadPoolExecutor(max_workers=6) as pool:
        codes = list(pool.map(run, tasks))
    (HERE/group/'exit-codes.json').write_text(json.dumps(codes)+'\n')
    if any(codes):
        print('Failures retained; inspect before extending campaign.', flush=True)


if __name__ == '__main__':
    main()
