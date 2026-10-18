"""Keep the viewer's packaged firmware aligned with the verified public release."""
import json,hashlib,shutil
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert json.loads((HERE/'acceptance.json').read_text())['accepted']
for name in ['local-browser.json','live-browser.json','live-hashes.json']:assert json.loads((HERE/name).read_text())['passed']
assert not (HERE/'packaged-promotion.json').exists()
for r in json.loads((HERE/'original/manifest.json').read_text()):assert sha(Path(r['path']))==r['sha256'],r['path']
current=json.loads((ROOT/'viewer/assets/runtime/highdof.json').read_text())
release=json.loads((HERE/'release/wasm/highdof.json').read_text())
assert {k:v for k,v in current.items() if k!='defaults'}=={k:v for k,v in release.items() if k!='defaults'}
def params(text):return dict(line.split()[:2] for line in text.splitlines() if line.strip() and not line.lstrip().startswith('#'))
assert params(current['defaults'])==params(release['defaults'])
changes=[]
for name in ['arduplane.js','arduplane.wasm','build.json','highdof.json','lqi-filter-tune.patch','lqi-filter-design.json']:
    dest=ROOT/'viewer'/('generated/ardupilot' if name.startswith('arduplane.') else 'assets/runtime')/name
    before=sha(dest) if dest.exists() else None
    shutil.copyfile(HERE/'release/wasm'/name,dest)
    changes.append(dict(path=str(dest.relative_to(ROOT)),before=before,after=sha(dest)))
(HERE/'packaged-promotion.json').write_text(json.dumps(dict(changes=changes,
    note='Packaged viewer matches verified public artifacts. Existing ArduPilot research checkout and its zero-trim gains remain preserved; rebuild public firmware from the recorded base plus bundled patch.'),indent=2)+'\n')
print(json.dumps(changes,indent=2))
