"""Align packaged viewer assets only after public release verification."""
import json,shutil,os
from pathlib import Path
from build import HERE,ROOT,sha
D=HERE/os.environ.get('RELEASE_DIR','deployment-v2');OUT=HERE/os.environ.get('RELEASE_OUTPUT','release');checks=HERE/os.environ.get('CHECK_DIR','.')
for name in ['local-browser.json','live-browser.json','live-hashes.json']:assert json.loads((checks/name).read_text())['passed']
assert json.loads((D/'qualification.json').read_text())['accepted']
assert json.loads((D/('disturbance.json' if os.environ.get('RELEASE_RATE')=='35' else 'fresh/qualification.json')).read_text())['accepted']
for r in json.loads((D/'original/manifest.json').read_text()):assert sha(Path(r['path']))==r['sha256'],r['path']
current=json.loads((ROOT/'viewer/assets/runtime/highdof.json').read_text());release=json.loads((OUT/'wasm/highdof.json').read_text())
assert {k:v for k,v in current.items() if k!='defaults'}=={k:v for k,v in release.items() if k!='defaults'}
if os.environ.get('RELEASE_RATE')=='35':
    def params(s):return dict(line.split()[:2] for line in s.splitlines() if line.strip() and not line.lstrip().startswith('#'))
    a=params(current['defaults']);b=params(release['defaults']);allowed={'CP3_RMAX','PG_TURN_RMAX'}
    assert {k:v for k,v in a.items() if k not in allowed}=={k:v for k,v in b.items() if k not in allowed}
    assert all(b[k]=='35' for k in allowed)
else:assert release['defaults'].startswith(current['defaults'])
changes=[]
for name in ['arduplane.js','arduplane.wasm','build.json','highdof.json','lqi-feasible-turns.patch','lqi-feasible-turns-validation.json']:
    dest=ROOT/'viewer'/('generated/ardupilot' if name.startswith('arduplane.') else 'assets/runtime')/name
    before=sha(dest) if dest.exists() else None;shutil.copyfile(OUT/'wasm'/name,dest)
    changes.append(dict(path=str(dest.relative_to(ROOT)),before=before,after=sha(dest)))
(D/'packaged-promotion.json').write_text(json.dumps(changes,indent=2));print(json.dumps(changes))
