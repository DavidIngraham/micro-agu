"""Run archived binaries with reversible viewer instrumentation; no AP source edits."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
VIEW = HERE.parents[1]/'viewer'
OUT = HERE/'holistic/transients-valid'
TARGETS = [VIEW/'generated/ardupilot/arduplane.js', VIEW/'generated/ardupilot/arduplane.wasm',
           VIEW/'assets/runtime/build.json', VIEW/'assets/runtime/highdof.json',
           VIEW/'packages/sim-browser/src/physics/physics.ts']
HARNESS = VIEW/'tests/integration/holistic-transient.ts'


def main():
    OUT.mkdir(exist_ok=True, parents=True)
    backup = OUT/'original'
    backup.mkdir()  # Never overwrite a prior snapshot.
    if HARNESS.exists():
        raise RuntimeError('Refuse to overwrite existing harness')
    originals = {p: p.read_bytes() for p in TARGETS}
    for p, data in originals.items():
        (backup/p.name).write_bytes(data)
    installed = dict(originals)
    records = []
    try:
        physics = (HERE/'physics.ts.experiment.txt').read_bytes()
        TARGETS[-1].write_bytes(physics)
        installed[TARGETS[-1]] = physics
        HARNESS.write_bytes((HERE/'transient_case.ts').read_bytes())
        for design, variants in [('fixed-design', ['standard','airspeed','sensor']), ('damping-design',['damping'])]:
            runtime = HERE/design/'runtime'
            for p in TARGETS[:-1]:
                data = (runtime/p.name).read_bytes()
                p.write_bytes(data)
                installed[p] = data
            jobs = []
            for variant in variants:
                for case in [1,3,13]:
                    spec = json.loads((HERE/f'fixed/sensor/cases/{case:02}.json').read_text())
                    spec['parameters'].update(AIRSPEED_CRUISE=5, AIRSPEED_MIN=5, AIRSPEED_MAX=7)
                    spec.update(variant=variant, protocol='settled steps and isolated gusts', speedup=8)
                    directory = OUT/variant
                    directory.mkdir(exist_ok=True)
                    path = directory/f'{case:02}.json'
                    casepath = directory/f'{case:02}-case.json'
                    casepath.write_text(json.dumps(spec,indent=2)+'\n')
                    jobs.append((casepath,path,variant))
            def run(job):
                casepath,path,variant = job
                env = dict(os.environ, PROPRIO_CASE='none' if variant in ['standard','airspeed'] else 'sensor')
                scheme = 'baseline' if variant=='standard' else 'lqi'
                with path.with_suffix('.log').open('w') as log:
                    try:
                        code = subprocess.run([str(VIEW/'node_modules/.bin/tsx'), str(HARNESS), str(casepath), scheme, str(path)],
                            cwd=VIEW,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=360).returncode
                    except subprocess.TimeoutExpired:
                        code = 124
                result = dict(path=str(path.relative_to(HERE)),exit_code=code,
                    firmware_sha256=hashlib.sha256((runtime/'arduplane.wasm').read_bytes()).hexdigest())
                print(result,flush=True)
                return result
            with ThreadPoolExecutor(max_workers=6) as pool:
                records.extend(pool.map(run,jobs))
            (OUT/'runs.json').write_text(json.dumps(records,indent=2)+'\n')
            if any(r['exit_code'] for r in records):
                raise RuntimeError('Campaign failure retained; inspect before extending')
    finally:
        conflicts = []
        for p, data in originals.items():
            if p.read_bytes() != installed[p]:
                conflicts.append(str(p))
                continue
            p.write_bytes(data)
            assert p.read_bytes() == data
        if HARNESS.exists() and HARNESS.read_bytes() == (HERE/'transient_case.ts').read_bytes():
            HARNESS.unlink()
        (OUT/'restoration.json').write_text(json.dumps(dict(conflicts=conflicts,
            original_sha256={str(p.relative_to(VIEW)):hashlib.sha256(data).hexdigest() for p,data in originals.items()}),indent=2)+'\n')
        if conflicts:
            raise RuntimeError('Concurrent changes preserved: '+str(conflicts))


if __name__=='__main__':
    main()
