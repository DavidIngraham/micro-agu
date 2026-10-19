import hashlib,json,tarfile
from pathlib import Path
from assess import assess
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
m=json.loads((P/'evidence-manifest.json').read_text())
assert sha((P/m['archive']).read_bytes())==m['sha256']
with tarfile.open(P/m['archive']) as t:
 assert set(t.getnames())==set(m['files'])
 for name,digest in m['files'].items():assert sha(t.extractfile(name).read())==digest,name
rows=json.loads((P/'metrics.json').read_text());assert len(rows)==72
assert sum(r['complete'] for r in rows)==68
for r in rows:
 assert r==assess(P/'runs'/f"{r['spec']['id']}.json"),r['spec']['id']
 assert r['gain_file_loaded'] and r['ekf_active']
 assert not r['active_before_power_ready']
 assert r['assumed_active_seconds']<=45.2
 if r['spec']['strategy']=='zero':assert r['screen_pass']
 if r['spec']['strategy'] in ['zero','weighted']:assert r['active_during_takeoff_stage']
for name in ['timeout-zero','timeout-weighted','smooth-timeout-weighted']:
 s=json.loads((P/'runs'/f'{name}.json').read_text())['samples']
 assert all(not x['active'] for x in s if 52<x['elapsed']<65)
 assert all(x['active'] for x in s if 70<x['elapsed']<75)
s=json.loads((P/'runs/late-valid-weighted.json').read_text())['samples']
assert all(x['active'] for x in s if 49<x['elapsed']<55)
assert any(.2<x['startup'].get('TKWQ',1)<.9 for x in s if 51<x['elapsed']<54)
for a,b in zip(s,s[1:]):
 if 51<a['elapsed']<54:assert 0<=b['startup']['TKWQ']-a['startup']['TKWQ']<.06
for file,field in [('controllers/lqi-gains.bin','gain_sha256'),('configurations/browser-physics.json','configuration_sha256')]:
 assert sha((ROOT/'aircraft/microagu'/file).read_bytes())==m[field]
assert sha((ROOT/'viewer/generated/physics/sim_wasm_bg.wasm').read_bytes())==m['physics_sha256']
for prefix in ['', 'initial-']:
 d=json.loads((P/f'{prefix}build-manifest.json').read_text())
 assert sha((P/f'{prefix}startup.patch').read_bytes())==d['patch_sha256']
print('PASS: 72 trace checksums, recomputed metrics, startup ownership, 45-second cap, timeout recovery, late-validity blend and release artifacts')
