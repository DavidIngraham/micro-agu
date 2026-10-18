"""Run unchanged prior campaign scenarios against the currently built firmware."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
VIEWER = ROOT / 'viewer'
PREVIOUS = HERE.parent / 'controller-stability-2026-10-16'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('iteration')
    parser.add_argument('--cases', default='0,1,3,9,15')
    parser.add_argument('--both', action='store_true')
    args = parser.parse_args()
    destination = HERE / args.iteration
    (destination / 'results').mkdir(parents=True, exist_ok=True)
    shutil.copyfile(VIEWER / 'assets/runtime/build.json', destination / 'build.json')
    shutil.copyfile(VIEWER / 'tests/integration/stability-case.ts', destination / 'stability-case.ts.txt')
    ids = range(16) if args.cases == 'all' else [int(x) for x in args.cases.split(',')]
    jobs = [(i, scheme) for i in ids for scheme in (['baseline', 'lqi'] if args.both else ['lqi'])]

    def run(job):
        i, scheme = job
        output = destination / 'results' / f'{i:02}-{scheme}.json'
        with output.with_suffix('.log').open('w') as log:
            result = subprocess.run([str(VIEWER / 'node_modules/.bin/tsx'),
                                     'tests/integration/stability-case.ts',
                                     str(PREVIOUS / 'cases' / f'{i:02}.json'), scheme, str(output)],
                                    cwd=VIEWER, stdout=log, stderr=subprocess.STDOUT, timeout=330)
        return i, scheme, result.returncode

    with ThreadPoolExecutor(max_workers=4) as pool:
        for result in pool.map(run, jobs):
            print(result, flush=True)


if __name__ == '__main__':
    main()
