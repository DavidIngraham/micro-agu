"""Verify restoration, fixed observer/model, exact runtime and matched gust incidence."""
import json,hashlib,os,subprocess,tempfile
from pathlib import Path
import numpy as np
from synthesize import arrays, BASE
HERE=Path(__file__).resolve().parent;BACK=HERE.parent/'lqi-backport-2026-10-17'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    original=json.loads((HERE/'original/manifest.json').read_text())
    for r in original:assert sha(Path(r['path']))==r['sha256'],r['path']
    for name in ['restoration.json','stage2-restoration.json','selection-restoration.json','endurance-restoration.json']:assert not json.loads((HERE/name).read_text())['conflicts']
    base=arrays(BASE)
    for d in (HERE/'designs').iterdir():
        generated=arrays((d/'AP_CustomControl_LQI_gains.h').read_text())
        for key in base:
            if key not in ['K','AW']:assert np.array_equal(base[key],generated[key]),(d.name,key)
        runtime=d/'runtime'
        for name in ['SIM_JSON.h','SIM_JSON.cpp']:
            assert (runtime/name).read_bytes()==(BACK/'runtime/deployed'/name).read_bytes()
        assert b'LQIP,' not in (runtime/'arduplane.wasm').read_bytes()
    count=0;complete=0;errors=[]
    frozen=(HERE/'selection-with-speed-task.json').stat().st_mtime_ns
    assert all(frozen<p.stat().st_mtime_ns for p in (HERE/'validation').glob('*/*.json') if p.stem.isdigit()),'Selection was not frozen before holdouts'
    selected=json.loads((HERE/'selection.json').read_text())['selected']
    patch=BACK/'deployed-filter.patch' if selected=='filtered' else HERE/'designs'/selected/'published-source.patch'
    runtime=BACK/'runtime/deployed' if selected=='filtered' else HERE/'designs'/selected/'runtime'
    ap=HERE.parents[2]/'ardupilot-browser-9dof'
    with tempfile.TemporaryDirectory(prefix='lqi-source-check-') as tmp:
        env=dict(os.environ,GIT_INDEX_FILE=str(Path(tmp)/'index'))
        subprocess.run(['git','read-tree','010ada6a4f0b26910fc18ecca176d2e65c5ed9d7'],cwd=ap,env=env,check=True)
        subprocess.run(['git','apply','--cached',str(patch)],cwd=ap,env=env,check=True)
        for name in ['AP_CustomControl.cpp','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI_gains.h']:
            assert subprocess.check_output(['git','show',':libraries/AP_CustomControl/'+name],cwd=ap,env=env)==(runtime/name).read_bytes()
    for phase in ['training','validation','missions','endurance']:
        for record in (HERE/phase).glob('*/runs.json'):
            name=record.parent.name
            runtime=BACK/'runtime'/('local' if name=='local' else 'deployed') if name in ['published','filtered','local'] else HERE/'designs'/name/'runtime'
            for run in json.loads(record.read_text()):
                assert run['firmware_sha256']==sha(runtime/'arduplane.wasm')
                d=json.loads((record.parent/f'{run["case"]:02}.json').read_text());count+=1;complete+=int(d['complete'])
                assert d['build']['sha256']['arduplane.wasm']==run['firmware_sha256']
                assert not d['diagnostics']
                p=d['spec']['parameters'];assert p['ARSPD_TYPE']==p['ARSPD_USE']==0
                assert p['CP3_HRTC']==(0 if name=='published' else .7)
                assert not any('Unknown parameter' in line for line in d['logs'])
                if phase in ['training','validation'] and d['complete']:
                    ss=d['samples'];i=next(i for i,s in enumerate(ss) if s['time']>=260 and np.linalg.norm(s['wind'])>1)
                    x,y,z,w=ss[i-1]['truth'][3:7];yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
                    wn,we,_=ss[i]['wind'];from_angle=np.arctan2(-we,-wn)
                    err=float(np.rad2deg(np.angle(np.exp(1j*(from_angle-yaw-np.pi/2)))))
                    errors.append(abs(err));assert abs(err)<8,(name,run['case'],err)
    out=dict(runs=count,completed=complete,restored_files=len(original),fixed_observer_and_model=True,
       runtime_hashes_verified=True,source_patch_reproduces_compiled_sources=True,no_extra_sensors=True,selection_precedes_holdouts=True,max_gust_incidence_error_deg=max(errors))
    (HERE/'audit.json').write_text(json.dumps(out,indent=2));print(json.dumps(out))
if __name__=='__main__':main()
