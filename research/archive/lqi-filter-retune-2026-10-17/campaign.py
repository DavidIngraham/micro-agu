"""AI-assisted published-tune retuning; snapshots protect unrelated local work."""
import difflib
import importlib.util
import json
from pathlib import Path
import subprocess
import numpy as np
from synthesize import CANDIDATES, main as synthesize
from evaluate import select, report
HERE=Path(__file__).resolve().parent;BACK=HERE.parent/'lqi-backport-2026-10-17'
spec=importlib.util.spec_from_file_location('runner',HERE.parent/'lqi-balanced-2026-10-17/campaign.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
b.HERE=HERE;b.HARNESS=b.VIEW/'tests/integration/retune-case.ts';b.MISSION=b.VIEW/'tests/integration/retune-mission.ts'
def main():
    synthesize()
    changed=subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=b.AP,text=True).splitlines()
    files=list(dict.fromkeys(b.FILES+[b.AP/p for p in changed if not p.startswith('modules/')]))
    original={p:p.read_bytes() for p in files};expected=dict(original)
    backup=HERE/'original';backup.mkdir();manifest=[]
    for i,(p,data) in enumerate(original.items()):
        (backup/str(i)).write_bytes(data);manifest.append(dict(path=str(p),backup=str(i),sha256=b.sha(p)))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    def write(p,data):
        data=data.encode() if isinstance(data,str) else data;p.write_bytes(data);expected[p]=data
    def install(runtime):
        for p in b.FILES[7:10]:write(p,(runtime/p.name).read_bytes())
    old=json.loads((BACK/'protocol.json').read_text());train=old['training']+[s for s in old['validation'] if s['id'] in [80,83,87]]
    rng=np.random.default_rng(2026101723);valid=[]
    for i in range(100,112):
        s=json.loads(json.dumps(train[0]));s['id']=i
        s['scales']={k:float(rng.uniform(.7,1.3)) for k in ['Cmq','joint_c_pitch','joint_c_roll','joint_c_yaw','CLbrake','CDbrake']}
        s['brakeRateScale']=float(rng.uniform(.7,1.1));s['wind']=float(rng.uniform(0,2));s['from']=float(rng.uniform(0,360))
        s['weather'].update(turbulence=float(rng.uniform(0,.6)),thermalStrength=float(rng.uniform(0,2)),seed=int(rng.integers(1,2**32-1)))
        valid.append(s)
    (HERE/'protocol.json').write_text(json.dumps(dict(training=train,validation=valid,candidates=CANDIDATES,
      filter_tau_s=.7,reference_limits_m_s=[-.5,.5],observer='published unchanged',sensors='original, no pitot or canopy truth',
      crosswind='2 m/s from aircraft right at onset, held fixed in earth frame for 5 s; corrects the prior earth-bearing confound',
      selection='Six previously seen training cases only. Freeze selection before twelve new Monte Carlo cases and AUTO/CRUISE missions.',
      gates='All operational screens; score improves >=5%; no median task loss >20%, acquisition time >10%, energy/power >5%; no individual score >20% regression or worst-quarter score regression; mission tracking <=20% with 1m floor.'),indent=2))
    def run(phase,name,cases,runtime):
        install(runtime);ss=json.loads(json.dumps(cases))
        for s in ss:s['parameters']=dict(CP3_HRTC=0 if name=='published' else .7,ARSPD_TYPE=0,ARSPD_USE=0)
        b.run_cases(phase,name,ss,runtime,mission=phase=='missions')
    try:
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:assert not p.exists();p.write_bytes((HERE/name).read_bytes())
        for name in ['published','filtered']:run('training',name,train,BACK/'runtime/deployed')
        # Exactly the public source base, then the already tested reference filter.
        for p in files:
            if p.is_relative_to(b.AP):write(p,subprocess.check_output(['git','show','HEAD:'+str(p.relative_to(b.AP))],cwd=b.AP))
        for p in b.FILES[2:5]:write(p,(BACK/'runtime/deployed'/p.name).read_bytes())
        for name in CANDIDATES:
            out=HERE/'designs'/name;write(b.FILES[5],(out/'AP_CustomControl_LQI_gains.h').read_bytes())
            with (out/'build.log').open('w') as log:
                rc=subprocess.run(['bash',str(b.VIEW/'tools/build/build_sitl_wasm.sh'),str(b.AP)],cwd=b.ROOT,env=b.ENV,stdout=log,stderr=subprocess.STDOUT).returncode
            for p in b.FILES[7:11]:expected[p]=p.read_bytes()
            if rc:raise RuntimeError('Build failed '+name)
            runtime=out/'runtime';runtime.mkdir()
            for p in b.FILES:(runtime/p.name).write_bytes(p.read_bytes())
            patch=''
            for p in b.FILES[2:6]:
                rel=str(p.relative_to(b.AP));before=subprocess.check_output(['git','show','HEAD:'+rel],cwd=b.AP,text=True)
                patch+=''.join(difflib.unified_diff(before.splitlines(True),p.read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
            (out/'published-source.patch').write_text(patch)
            (runtime/'hashes.json').write_text(json.dumps({p.name:b.sha(p) for p in runtime.iterdir()},indent=2))
            run('training',name,train,runtime)
        chosen=select()
        if chosen:
            selected_runtime=BACK/'runtime/deployed' if chosen=='filtered' else HERE/'designs'/chosen/'runtime'
            for name,runtime in [('published',BACK/'runtime/deployed'),('local',BACK/'runtime/local'),(chosen,selected_runtime)]:
                run('validation',name,valid,runtime);run('missions',name,valid,runtime)
        report()
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:
            if p.exists() and p.read_bytes()==(HERE/name).read_bytes():p.unlink()
        (HERE/'restoration.json').write_text(json.dumps(dict(conflicts=conflicts,verified=len(original)-len(conflicts)),indent=2))
        assert not conflicts,conflicts
if __name__=='__main__':main()
