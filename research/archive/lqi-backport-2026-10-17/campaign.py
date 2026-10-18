"""AI-assisted filter backport: existing observers/gains, no extra sensors."""
import difflib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('prior',HERE.parent/'lqi-balanced-2026-10-17/campaign.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
b.HERE=HERE;b.HARNESS=b.VIEW/'tests/integration/backport-case.ts';b.MISSION=b.VIEW/'tests/integration/backport-mission.ts'

def filtered(cpp,h,front):
    cpp,h,front=b.governor(cpp,h,front)
    h=b.replace(h,'    AP_Float _height_tc;','    AP_Float _height_tc;\n    AP_Float _height_rate_tc;\n    float _smoothed_climb;')
    cpp=b.replace(cpp,'    AP_GROUPEND','''    // @Param: HRTC
    // @DisplayName: LQI climb reference smoothing
    // @Description: Time constant for climb reference smoothing before height integration and feedforward. Zero disables smoothing.
    // @Units: s
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("HRTC", 4, AP_CustomControl_LQI, _height_rate_tc, 0.0f),
    AP_GROUPEND''')
    cpp=b.replace(cpp,'    return isfinite(_height_up.get())','    return isfinite(_height_rate_tc.get()) && _height_rate_tc >= 0 && _height_rate_tc <= 3 &&\n        isfinite(_height_up.get())')
    cpp=b.replace(cpp,'    _height_reference = 0;','    _height_reference = 0;\n    _smoothed_climb = 0;')
    cpp=b.replace(cpp,'    const float governed_climb = constrain_float(climb, -_height_down.get(), _height_up.get());', '''    const float requested_climb = constrain_float(climb, -_height_down.get(), _height_up.get());
    _smoothed_climb += DT/(_height_rate_tc+DT)*(requested_climb-_smoothed_climb);
    const float governed_climb = _smoothed_climb;''')
    return cpp,h,front

def main():
    # Preserve all existing local work. Temporarily test published HEAD separately.
    changed=subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=b.AP,text=True).splitlines()
    changed=[b.AP/p for p in changed if not p.startswith('modules/')]
    files=list(dict.fromkeys(b.FILES+changed))
    original={p:p.read_bytes() for p in files};expected=dict(original)
    backup=HERE/'original';backup.mkdir()
    manifest=[]
    for i,(p,data) in enumerate(original.items()):
        (backup/str(i)).write_bytes(data);manifest.append(dict(path=str(p),backup=str(i),sha256=b.sha(p)))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    def write(p,data):
        data=data.encode() if isinstance(data,str) else data;p.write_bytes(data);expected[p]=data
    def install(runtime):
        for p in b.FILES[7:10]:write(p,(runtime/p.name).read_bytes())
    train=[json.loads((HERE.parent/f'controller-stability-2026-10-16/cases/{i:02}.json').read_text()) for i in [1,3,13]]
    rng=np.random.default_rng(2026101711);valid=[]
    for i in range(80,88):
        s=json.loads(json.dumps(train[0]));s['id']=i
        s['scales']={k:float(rng.uniform(.7,1.3)) for k in ['Cmq','joint_c_pitch','joint_c_roll','joint_c_yaw','CLbrake','CDbrake']}
        s['brakeRateScale']=float(rng.uniform(.7,1.1));s['wind']=float(rng.uniform(0,2));s['from']=float(rng.uniform(0,360))
        s['weather'].update(turbulence=float(rng.uniform(0,.6)),thermalStrength=float(rng.uniform(0,2)),seed=int(rng.integers(1,2**32-1)))
        valid.append(s)
    (HERE/'protocol.json').write_text(json.dumps(dict(training=train,validation=valid,
        intervention='Only CP3_HRTC: 0 vs 0.7. Existing 0.5 m/s rate limits, T=2, existing gains and observer. ARSPD_TYPE=0, no truth proprioception.',
        acceptance='All operational screens; >=5% paired median score improvement; no task median loss >20%, acquisition >10%, energy >5%, mission tracking >20% with 1m floor. No candidate retuning.'),indent=2))
    try:
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:
            assert not p.exists();p.write_bytes((HERE/name).read_bytes())
        for cohort in ['local','deployed']:
            for p in files:
                if p.is_relative_to(b.AP):
                    data=original[p] if cohort=='local' else subprocess.check_output(['git','show','HEAD:'+str(p.relative_to(b.AP))],cwd=b.AP)
                    write(p,data)
            paths=[b.FILES[4],b.FILES[3],b.FILES[2]]
            before=[p.read_text() for p in paths]
            after=filtered(*before)
            patch=''
            for p,old,new in zip(paths,before,after):
                write(p,new);rel=str(p.relative_to(b.AP))
                patch+=''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
            (HERE/f'{cohort}-filter.patch').write_text(patch)
            with (HERE/f'{cohort}-build.log').open('w') as log:
                rc=subprocess.run(['bash',str(b.VIEW/'tools/build/build_sitl_wasm.sh'),str(b.AP)],cwd=b.ROOT,env=b.ENV,stdout=log,stderr=subprocess.STDOUT).returncode
            for p in b.FILES[7:11]:expected[p]=p.read_bytes()
            if rc:raise RuntimeError('build '+cohort)
            runtime=HERE/'runtime'/cohort;runtime.mkdir(parents=True)
            for p in b.FILES:(runtime/p.name).write_bytes(p.read_bytes())
            (runtime/'hashes.json').write_text(json.dumps({p.name:b.sha(p) for p in runtime.iterdir()},indent=2))
            for phase,cases in [('training',train),('validation',valid),('missions',valid)]:
                for variant,tau in [('off',0),('on',.7)]:
                    ss=json.loads(json.dumps(cases))
                    for s in ss:s['parameters']=dict(CP3_HRTC=tau,ARSPD_TYPE=0,ARSPD_USE=0)
                    b.run_cases(phase,cohort+'_'+variant,ss,runtime,mission=phase=='missions')
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
