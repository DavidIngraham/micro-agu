"""Verify unchanged gains, original sensing, firmware identity and restoration."""
import hashlib
import json
from pathlib import Path
import subprocess
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2];AP=ROOT.parent/'ardupilot-browser-9dof'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((HERE/'original/manifest.json').read_text())
for r in manifest:assert sha(Path(r['path']))==r['sha256'],r['path']
assert not json.loads((HERE/'restoration.json').read_text())['conflicts']
gain='libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h'
old=next(r for r in manifest if r['path']==str(AP/gain))
assert sha(HERE/'runtime/local/AP_CustomControl_LQI_gains.h')==old['sha256']
assert (HERE/'runtime/deployed/AP_CustomControl_LQI_gains.h').read_bytes()==subprocess.check_output(['git','show','HEAD:'+gain],cwd=AP)
for cohort in ['local','deployed']:
    runtime=HERE/'runtime'/cohort
    assert b'LQIP,' not in (runtime/'arduplane.wasm').read_bytes()
    assert b'LQID,' not in (runtime/'arduplane.wasm').read_bytes()
    assert 'research_relative' not in (runtime/'SIM_JSON.h').read_text()
    assert 'L_PROP' not in (runtime/'AP_CustomControl_LQI.cpp').read_text()
count=0;complete=0
site=ROOT.parent/'microagu-pages-publish/assets/microagu-viewer/wasm'
for local,name in [(ROOT/'viewer/assets/runtime/highdof.json','highdof.json'),
                   (ROOT/'viewer/generated/physics/sim_wasm_bg.wasm','sim_wasm_bg.wasm')]:
    assert sha(local)==sha(site/name),f'Study physics differs from deployed {name}'
for phase in ['training','validation','missions']:
    for record in (HERE/phase).glob('*/runs.json'):
        for run in json.loads(record.read_text()):
            cohort,variant=run['variant'].split('_')
            assert run['firmware_sha256']==sha(HERE/'runtime'/cohort/'arduplane.wasm')
            p=record.parent/f'{run["case"]:02}.json';d=json.loads(p.read_text());count+=1;complete+=int(d['complete'])
            assert not any('Unknown parameter' in line for line in d['logs'])
            params=d['spec']['parameters'];assert params['ARSPD_TYPE']==0 and params['ARSPD_USE']==0
            assert params['CP3_HRTC']==(0 if variant=='off' else .7)
assert count==76 and complete==76,(count,complete)
out=dict(restored_files=len(manifest),runs=count,complete=complete,gains_unchanged=True,no_added_sensors=True,
         physics_and_config_match_deployed=True)
(HERE/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
