import hashlib,json,tarfile
from pathlib import Path
import numpy as np
from assess import assess
P=Path(__file__).resolve().parent;ROOT=P.parents[2]
sha=lambda b:hashlib.sha256(b).hexdigest()
manifest=json.loads((P/'evidence-manifest.json').read_text())
assert sha((P/manifest['archive']).read_bytes())==manifest['sha256']
with tarfile.open(P/manifest['archive']) as tar:
 assert set(tar.getnames())==set(manifest['files'])
 for name,digest in manifest['files'].items():
  assert sha(tar.extractfile(name).read())==digest,name
runtime=json.loads((P/'runtime-manifest.json').read_text())
for name,digest in runtime['source_hashes'].items():assert sha((ROOT/name).read_bytes())==digest,name
assert sha((ROOT/'viewer/generated/ardupilot/arduplane.wasm').read_bytes())==runtime['firmware_sha256']
assert sha((ROOT/'aircraft/microagu/controllers/lqi-gains.bin').read_bytes())==runtime['gains_sha256']
assert sha((ROOT/'aircraft/microagu/configurations/browser-physics.json').read_bytes())==runtime['configuration_sha256']
rows=json.loads((P/'metrics.json').read_text());assert len(rows)==56
assert all(r['gain_file_loaded'] and r['ekf_active'] for r in rows)
assert sum(r['complete'] for r in rows)==52
for r in rows:
 actual=assess(P/'runs'/f"{r['id']}.json")
 assert actual==r,r['id']
for r in json.loads((P/'calibration-summary.json').read_text()):
 assert abs(r['yaw_rms']-20)<.1
 assert abs(r['pitch_yaw_ratio']-.4)<.003
 assert abs(r['phase']-r['spec']['target_phase']*180/np.pi)<.3
for r in json.loads((P/'numerical-check.json').read_text()):assert r['max_rate_difference_deg_s']<.02
assert json.loads((P/'calibration-basis.json').read_text())['parity']==0
print('Passed: 62 evidence checksums, source/release provenance, 56 recomputed metrics, calibration and numerical checks.')
