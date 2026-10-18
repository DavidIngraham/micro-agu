"""Compile the portable default-preserving patch, then restore exact originals."""
import importlib.util
import json
from pathlib import Path
import subprocess
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('previous',HERE.parent/'lqi-balanced-2026-10-17/campaign.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
assert (HERE/'restoration.json').exists(), 'Wait for the experiment to restore first'
assert not json.loads((HERE/'restoration.json').read_text())['conflicts']
original={p:p.read_bytes() for p in b.FILES};expected=dict(original)
rc=None
try:
    subprocess.run(['git','apply','--check',str(HERE/'reference-filter.patch')],cwd=b.AP,check=True)
    subprocess.run(['git','apply',str(HERE/'reference-filter.patch')],cwd=b.AP,check=True)
    for p in b.FILES[2:5]:expected[p]=p.read_bytes()
    with (HERE/'portable-filter-build.log').open('w') as log:
        rc=subprocess.run(['bash',str(b.VIEW/'tools/build/build_sitl_wasm.sh'),str(b.AP)],cwd=b.ROOT,env=b.ENV,
                          stdout=log,stderr=subprocess.STDOUT).returncode
    for p in b.FILES[7:11]:expected[p]=p.read_bytes()
finally:
    conflicts=[]
    for p,data in original.items():
        if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
        p.write_bytes(data)
    (HERE/'portable-filter-check.json').write_text(json.dumps(dict(build_exit_code=rc,conflicts=conflicts),indent=2))
    assert not conflicts,conflicts
assert rc==0,rc
print('Portable parameterized filter builds; original files restored.')
