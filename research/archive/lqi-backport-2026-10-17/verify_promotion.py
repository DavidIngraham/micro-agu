"""Verify intentional post-audit changes and an untouched publishing checkout."""
import hashlib
import json
from pathlib import Path
import subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2];AP=ROOT.parent/'ardupilot-browser-9dof'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
allowed={str(AP/'libraries/AP_CustomControl'/name) for name in
         ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h']}
allowed|={str(ROOT/path) for path in ['viewer/generated/ardupilot/arduplane.js',
    'viewer/generated/ardupilot/arduplane.wasm','viewer/assets/runtime/build.json']}
preserved=0
for r in json.loads((HERE/'original/manifest.json').read_text()):
    if r['path'] not in allowed:
        assert sha(Path(r['path']))==r['sha256'],r['path'];preserved+=1
meta=json.loads((ROOT/'viewer/assets/runtime/build.json').read_text())
for rel in ['viewer/generated/ardupilot/arduplane.js','viewer/generated/ardupilot/arduplane.wasm','viewer/assets/runtime/highdof.json']:
    p=ROOT/rel;assert sha(p)==meta['sha256'][p.name],rel
for rel,value in meta['controller_source_sha256'].items():assert sha(AP/rel)==value,rel
assert 'CP3_HRTC 0.7' in (ROOT/'simulation/mission-controller-candidate.parm').read_text()
assert 'CP3_HRTC 0.7' in json.loads((ROOT/'viewer/assets/runtime/highdof.json').read_text())['defaults']
assert (HERE/'live-before.json').read_bytes()==(HERE/'live-after.json').read_bytes()
site=ROOT.parent/'microagu-pages-publish'
assert not subprocess.check_output(['git','status','--porcelain'],cwd=site,text=True).strip()
out=dict(local_promoted=True,public_release=False,preserved_unrelated_snapshot_files=preserved,
         local_runtime_hashes_verified=True,public_metadata_unchanged=True,
         reason=json.loads((HERE/'acceptance.json').read_text())['deployed']['reasons'])
(HERE/'promotion-audit.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
