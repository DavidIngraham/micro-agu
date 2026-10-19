import hashlib,json,tarfile
from pathlib import Path
from assess import assess
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
m=json.loads((P/'evidence-manifest.json').read_text())
assert sha((P/m['archive']).read_bytes())==m['sha256']
with tarfile.open(P/m['archive']) as t:
 assert set(t.getnames())==set(m['files'])
 for name,digest in m['files'].items():assert sha(t.extractfile(name).read())==digest
rows=json.loads((P/'metrics.json').read_text());assert len(rows)==14
for r in rows:
 assert r==assess(P/'runs'/f"{r['spec']['id']}.json")
 assert r['complete'] and r['gain_file_loaded'] and r['ekf_active']
 assert not r['active_before_power_ready']
 assert r['min_free_height_m']>.1 and r['late_active_fraction']==1
 if r['spec']['variant']=='candidate' and 'headwind' not in r['spec']['id']:assert r['active_during_takeoff_stage']
 if 'headwind' in r['spec']['id']:assert not r['active_during_takeoff_stage']
d=json.loads((P/'runs/candidate-switch.json').read_text())
assert all(not s['active'] for s in d['samples'] if 10.5<s['elapsed']<13.5)
assert all(s['active'] for s in d['samples'] if 14.5<s['elapsed']<17)
d=json.loads((P/'runs/candidate-headwind-probe.json').read_text())
assert all(not(s['eligibilityFlags']&2) for s in d['samples'] if s['takeoffStage'])
assert sha((ROOT/'aircraft/microagu/controllers/lqi-gains.bin').read_bytes())==m['gain_sha256']
assert sha((ROOT/'viewer/generated/physics/sim_wasm_bg.wasm').read_bytes())==m['physics_sha256']
assert sha((ROOT/'aircraft/microagu/configurations/browser-physics.json').read_bytes())==m['configuration_sha256']
print('PASS: fourteen runs, six actual LQI takeoffs, wind-validity fallback, switch behavior, provenance and checksums')
