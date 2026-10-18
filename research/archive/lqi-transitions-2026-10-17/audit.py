"""Verify restoration, firmware provenance, frozen selection and actual sensing."""
import hashlib
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((HERE/'original/manifest.json').read_text())
for item in manifest:assert sha(Path(item['path']))==item['sha256'],item['path']
assert not json.loads((HERE/'restoration.json').read_text())['conflicts']
for rel,value in json.loads((HERE/'physics-input-hashes.json').read_text()).items():
    assert sha(HERE.parents[1]/rel)==value,rel
count=0;complete=0
for phase in ['training','validation','missions','ablation']:
    for runfile in (HERE/phase).glob('*/runs.json'):
        for run in json.loads(runfile.read_text()):
            count+=1
            datafile=runfile.parent/f'{run["case"]:02}.json'
            d=json.loads(datafile.read_text());complete+=int(d['complete'])
            assert not any('Unknown parameter' in line for line in d['logs'])
            name='fixed' if run['variant']=='standard' else run['variant']
            if phase=='ablation':name=name.rsplit('_',1)[0]
            assert sha(HERE/'designs'/name/'runtime/arduplane.wasm')==run['firmware_sha256']
            assert run['exit_code']==0 or not d['complete']
            if phase in ['validation','missions']:assert datafile.stat().st_mtime>(HERE/'selection.json').stat().st_mtime
            if d['scheme']=='lqi' and d['complete']:
                assert all(row['values'][42]==1 for row in d['diagnostics'])
                assert all(abs(row['values'][4])<=1.00001 for row in d['diagnostics'])
                modes={int(line.split(',')[2]) for line in d['logs'] if line.startswith('LQIP,')}
                assert modes==({4} if d['spec'].get('sensor')=='truth' else {3}),modes
failed_screens=[]
for phase in ['training','validation','missions','ablation']:
    for row in json.loads((HERE/phase/'metrics.json').read_text()):
        failures=row.get('gates',row.get('screen_failures',[]))
        if failures:failed_screens.append(dict(phase=phase,variant=row['variant'],case=row['case'],failures=failures))
out=dict(restored_files=len(manifest),runs=count,complete=complete,firmware_and_sensor_verified=True,operational_screen_failures=failed_screens)
(HERE/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
