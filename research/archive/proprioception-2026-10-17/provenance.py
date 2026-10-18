"""Retain exact artifacts and verify that each run identifies an archived binary."""
import hashlib
import json
from pathlib import Path
import subprocess
from prepare import AP, ROOT, HERE, VIEW

patch = subprocess.check_output(['git', 'diff', '--binary', '--', '.', ':(exclude)modules'], cwd=AP)
(HERE/'ardupilot-working-tree.patch').write_bytes(patch)
for name in ['AP_NavEKF3_LowSpeed.h']:
    p = AP/'libraries/AP_NavEKF3'/name
    if p.exists():
        (HERE/name).write_bytes(p.read_bytes())
binaries = {hashlib.sha256(p.read_bytes()).hexdigest(): str(p.relative_to(HERE))
            for p in HERE.glob('*-design/runtime/arduplane.wasm')}
runs = []
for group in ['fixed', 'efficient', 'damping']:
    for p in (HERE/group).glob('*/results/*.json'):
        d = json.loads(p.read_text())
        expected = d['build']['sha256']['arduplane.wasm']
        if expected not in binaries:
            current = VIEW/'generated/ardupilot/arduplane.wasm'
            assert hashlib.sha256(current.read_bytes()).hexdigest() == expected
            dest = HERE/'additional-runtime'/expected
            dest.mkdir(parents=True, exist_ok=True)
            for name in ['arduplane.wasm', 'arduplane.js']:
                (dest/name).write_bytes((VIEW/'generated/ardupilot'/name).read_bytes())
            binaries[expected] = str((dest/'arduplane.wasm').relative_to(HERE))
        runs.append(dict(path=str(p.relative_to(HERE)), sha256=expected, complete=d['complete']))
report = dict(archived_binaries=binaries, runs=runs,
              experiment_source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in HERE.iterdir() if p.suffix in ['.py', '.ts', '.sh']},
              runtime_physics_sha256=hashlib.sha256((VIEW/'assets/runtime/highdof.json').read_bytes()).hexdigest())
(HERE/'provenance.json').write_text(json.dumps(report, indent=2)+'\n')
print('Verified archived firmware for', len(runs), 'runs')
