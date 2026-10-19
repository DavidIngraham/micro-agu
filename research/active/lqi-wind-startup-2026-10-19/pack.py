import hashlib,json,tarfile
from pathlib import Path
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
files=sorted((P/'runs').glob('*.json'))
initial=json.loads((P/'initial-build-manifest.json').read_text());final=json.loads((P/'build-manifest.json').read_text())
archive=P/'evidence.tar.gz'
with tarfile.open(archive,'w:gz',compresslevel=6) as t:
 for f in files:t.add(f,arcname=str(f.relative_to(P)))
m=dict(archive=archive.name,sha256=sha(archive.read_bytes()),files={str(f.relative_to(P)):sha(f.read_bytes()) for f in files},firmware_by_run={f.stem:(final if f.stem.startswith(('smooth-','late-valid-')) else initial)['firmware_sha256'] for f in files},gain_sha256=sha((ROOT/'aircraft/microagu/controllers/lqi-gains.bin').read_bytes()),physics_sha256=sha((ROOT/'viewer/generated/physics/sim_wasm_bg.wasm').read_bytes()),configuration_sha256=sha((ROOT/'aircraft/microagu/configurations/browser-physics.json').read_bytes()))
(P/'evidence-manifest.json').write_text(json.dumps(m,indent=2)+'\n')
rows=json.loads((P/'metrics.json').read_text())
summary=dict(runs=len(rows),complete=sum(r['complete'] for r in rows),screen_pass=sum(r['screen_pass'] for r in rows),failed=[r['spec']['id'] for r in rows if not r['screen_pass']],note='Counts include unequal scenarios, repeats and two firmware revisions; not failure probabilities.')
(P/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(summary,'archive bytes',archive.stat().st_size)
