"""AI-assisted matched-governor LQI study. No deployment or production tune changes.

This orchestrator snapshots all mutated files, builds archived-source candidates,
keeps exact firmware, runs training only before selection, then fresh validation.
"""
from concurrent.futures import ThreadPoolExecutor
import difflib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
import sys

import numpy as np
from scipy.linalg import solve_discrete_are

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PREV = HERE.parent/'proprioception-2026-10-17'
AP = ROOT.parent/'ardupilot-browser-9dof'
VIEW = ROOT/'viewer'
sys.path.insert(0,str(PREV))
from prepare import arrays_header

ENV = dict(os.environ, PATH='/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:/usr/local/bin:/usr/bin:/bin')
FILES = [AP/'libraries/SITL/SIM_JSON.h', AP/'libraries/SITL/SIM_JSON.cpp',
    AP/'libraries/AP_CustomControl/AP_CustomControl.cpp', AP/'libraries/AP_CustomControl/AP_CustomControl_LQI.h',
    AP/'libraries/AP_CustomControl/AP_CustomControl_LQI.cpp', AP/'libraries/AP_CustomControl/AP_CustomControl_LQI_gains.h',
    VIEW/'packages/sim-browser/src/physics/physics.ts', VIEW/'generated/ardupilot/arduplane.js',
    VIEW/'generated/ardupilot/arduplane.wasm', VIEW/'assets/runtime/build.json', VIEW/'assets/runtime/RUST-NOTICES.txt',
    AP/'libraries/AP_TECS/AP_TECS_Paraglider.cpp']
HARNESS = VIEW/'tests/integration/balanced-case.ts'
MISSION = VIEW/'tests/integration/balanced-mission.ts'
PARMS = dict(TECS_CLMB_MAX=1, TECS_SINK_MAX=2.5, CP3_HUP=1, CP3_HDN=1, CP3_HTC=2,
    ARSPD_TYPE=100, ARSPD_USE=1, ARSPD_SKIP_CAL=1, ARSPD_OFFSET=0, ARSPD_RATIO=1.99,
    SIM_ARSPD_RATIO=1.99, SIM_ARSPD_RND=2)
CANDIDATES = dict(fixed=dict(pitch=30, height=1, integral=3, throttle=1000, vertical=0),
    damped=dict(pitch=90, height=1, integral=3, throttle=1000, vertical=0),
    balanced=dict(pitch=90, height=2, integral=6, throttle=700, vertical=0),
    responsive=dict(pitch=90, height=4, integral=12, throttle=500, vertical=0),
    vertical=dict(pitch=90, height=2, integral=6, throttle=300, vertical=4))


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()


def replace(s,old,new):
    assert s.count(old)==1,old[:90]
    return s.replace(old,new)


def governor(cpp, header, frontend):
    header=replace(header,'    using AP_CustomControl_Backend::AP_CustomControl_Backend;',
        '    AP_CustomControl_LQI(AP_CustomControl &frontend, float dt);\n    static const AP_Param::GroupInfo var_info[];')
    header=replace(header,'private:\n', 'private:\n    AP_Float _height_up;\n    AP_Float _height_down;\n    AP_Float _height_tc;\n')
    definitions='''
// AI-assisted: configurable reference limits; defaults preserve the prior governor.
const AP_Param::GroupInfo AP_CustomControl_LQI::var_info[] = {
    // @Param: HUP
    // @DisplayName: LQI height reference climb limit
    // @Description: Maximum climb rate of the LQI internal altitude reference.
    // @Units: m/s
    // @Range: 0.1 3
    // @User: Advanced
    AP_GROUPINFO("HUP", 1, AP_CustomControl_LQI, _height_up, 0.5f),
    // @Param: HDN
    // @DisplayName: LQI height reference descent limit
    // @Description: Positive maximum descent rate of the LQI internal altitude reference.
    // @Units: m/s
    // @Range: 0.1 3
    // @User: Advanced
    AP_GROUPINFO("HDN", 2, AP_CustomControl_LQI, _height_down, 0.5f),
    // @Param: HTC
    // @DisplayName: LQI height reference time constant
    // @Description: Time constant for altitude reference approach near the requested height.
    // @Units: s
    // @Range: 0.2 10
    // @User: Advanced
    AP_GROUPINFO("HTC", 3, AP_CustomControl_LQI, _height_tc, 2.0f),
    AP_GROUPEND
};

AP_CustomControl_LQI::AP_CustomControl_LQI(AP_CustomControl &frontend, float dt) :
    AP_CustomControl_Backend(frontend, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
}
'''
    cpp=replace(cpp,'bool AP_CustomControl_LQI::valid',definitions+'\nbool AP_CustomControl_LQI::valid')
    cpp=replace(cpp,'return in.eligible &&', '''return isfinite(_height_up.get()) && _height_up >= 0.1f && _height_up <= 3.0f &&
        isfinite(_height_down.get()) && _height_down >= 0.1f && _height_down <= 3.0f &&
        isfinite(_height_tc.get()) && _height_tc >= 0.2f && _height_tc <= 10.0f &&
        in.eligible &&''')
    cpp=replace(cpp,'(in.reference.height_home_m-_height_origin-_height_reference)*0.5f;',
        '(in.reference.height_home_m-_height_origin-_height_reference)/_height_tc;')
    cpp=replace(cpp,'constrain_float(climb, -0.5f, 0.5f);',
        'constrain_float(climb, -_height_down.get(), _height_up.get());')
    frontend=replace(frontend,'    AP_GROUPEND\n', '''#if AP_PLANE_CUSTOMCONTROL_LQI_ENABLED
    // @Group: 3_
    // @Path: ./AP_CustomControl_LQI.cpp
    AP_SUBGROUPVARPTR(_backend, "3_", 8, AP_CustomControl, _backend_var_info[2]),
#endif

    AP_GROUPEND
''')
    frontend=replace(frontend,'        _backend = NEW_NOTHROW AP_CustomControl_LQI(*this, _dt);',
        '        _backend = NEW_NOTHROW AP_CustomControl_LQI(*this, _dt);\n        _backend_var_info[get_type()] = AP_CustomControl_LQI::var_info;')
    return cpp,header,frontend


def synthesize(name, cfg):
    out=HERE/'designs'/name; out.mkdir(parents=True,exist_ok=True)
    z=dict(np.load(PREV/'fixed-design/design.npz'))
    q=z['Q'].copy(); q[9,9]+=cfg['pitch']-30; q[12,12]+=cfg['pitch']-30
    q[:16,:16]+=(cfg['height']-1)*np.outer(z['H'][0],z['H'][0])
    q[16,16]=cfg['integral']; q[7,7]+=cfg['vertical']
    effort=z['R'].copy();effort[0,0]=cfg['throttle']
    a,b=z['AA'],z['BB']; p=solve_discrete_are(a,b,q,effort)
    k=np.linalg.solve(effort+b.T@p@b,b.T@p@a)
    poles=np.linalg.eigvals(a-b@k);assert max(abs(poles))<1
    aw=np.linalg.inv(z['US'][:,None]*k[:,16:19])
    header=(PREV/'fixed-design/AP_CustomControl_LQI_gains.h').read_text()
    def value(x):
        return '{'+', '.join(value(v) for v in x)+'}' if np.ndim(x) else f'{float(x):.9e}f'
    for key,arr in dict(K=k,AW=aw).items():
        pattern=r'(static constexpr float '+key+r'(?:\[\d+\])+ = )[^;]+;'
        header,n=re.subn(pattern,lambda m:m[1]+value(arr)+';',header);assert n==1
    z.update(K=k,Q=q,R=effort);np.savez(out/'design.npz',**z)
    header+=arrays_header(out)
    (out/'AP_CustomControl_LQI_gains.h').write_text(header)
    (out/'design.json').write_text(json.dumps(dict(**cfg,max_local_pole=float(max(abs(poles))),
        note='Same archived linearization/trim/observer; local DARE poles do not prove nonlinear stability.'),indent=2)+'\n')
    return out


def specs():
    train=[]
    for i in [1,3,13]:
        s=json.loads((PREV/f'fixed/sensor/cases/{i:02}.json').read_text());s['parameters'].update(PARMS);train.append(s)
    rng=np.random.default_rng(2026101702);valid=[]
    for i in range(40,48):
        s=json.loads(json.dumps(train[0]));s.update(id=i,mode='CRUISE')
        s['scales']={k:float(rng.uniform(.7,1.3)) for k in s['scales']}
        s['brakeRateScale']=float(rng.uniform(.7,1.1))
        s['wind']=float(rng.uniform(0,2));s['from']=float(rng.uniform(0,360))
        s['weather'].update(turbulence=float(rng.uniform(0,.6)),thermalStrength=float(rng.uniform(0,2)),seed=int(rng.integers(1,2**32-1)))
        valid.append(s)
    return train,valid


def run_cases(phase, variant, cases, runtime, mission=False):
    directory=HERE/phase/variant;directory.mkdir(parents=True,exist_ok=True)
    jobs=[]
    for source in cases:
        s=json.loads(json.dumps(source));s.update(variant=variant,speedup=8)
        if mission:s['mode']='AUTO' if s['id']%2==0 else 'CRUISE'
        path=directory/f'{s["id"]:02}.json';case=directory/f'{s["id"]:02}-case.json'
        assert not path.exists(),path
        case.write_text(json.dumps(s,indent=2)+'\n');jobs.append((case,path))
    def run(job):
        case,path=job
        env=dict(ENV,PROPRIO_CASE='none' if variant=='standard' else 'sensor')
        scheme='baseline' if variant=='standard' else 'lqi'
        with path.with_suffix('.log').open('w') as log:
            try:
                rc=subprocess.run([str(VIEW/'node_modules/.bin/tsx'),str(MISSION if mission else HARNESS),str(case),scheme,str(path)],
                    cwd=VIEW,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=400).returncode
            except subprocess.TimeoutExpired:rc=124
        result=dict(phase=phase,variant=variant,case=int(case.name.split('-')[0]),exit_code=rc,
                    firmware_sha256=sha(runtime/'arduplane.wasm'))
        print(json.dumps(result),flush=True)
        return result
    with ThreadPoolExecutor(max_workers=6) as pool:results=list(pool.map(run,jobs))
    (directory/'runs.json').write_text(json.dumps(results,indent=2)+'\n')
    # Flight failures stay in the dataset; infrastructure failures are fatal.
    if any(not p.exists() for _,p in jobs):raise RuntimeError('Missing run result')


def main():
    backup=HERE/'original';backup.mkdir()
    assert not HARNESS.exists() and not MISSION.exists()
    original={p:p.read_bytes() for p in FILES};expected=dict(original)
    manifest=[]
    for i,(p,b) in enumerate(original.items()):
        (backup/str(i)).write_bytes(b);manifest.append(dict(path=str(p),backup=str(i),sha256=hashlib.sha256(b).hexdigest()))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    def write(p,data):
        b=data.encode() if isinstance(data,str) else data;p.write_bytes(b);expected[p]=b
    def install_runtime(runtime):
        for p in FILES[7:10]:write(p,(runtime/p.name).read_bytes())
    try:
        HARNESS.write_bytes((HERE/'case.ts').read_bytes());MISSION.write_bytes((HERE/'mission.ts').read_bytes())
        for p in FILES[:2]:write(p,(PREV/(p.name+'.experiment.txt')).read_bytes())
        write(FILES[6],(PREV/'physics.ts.experiment.txt').read_bytes())
        # Only limit the standard controller's reference trajectory. Changing
        # TECS_SINK_MAX also rescales its throttle feedforward and is confounded.
        tecs=original[FILES[-1]].decode()
        limited=replace(tecs,'const float dn_step_max = MAX(_sink_rate_limit, 0.0f) * _DT;',
            'const float dn_step_max = MIN(MAX(_sink_rate_limit, 0.0f), 1.0f) * _DT; // SITL study reference cap')
        write(FILES[-1],limited)
        (HERE/'standard-reference-only.patch').write_text(''.join(difflib.unified_diff(tecs.splitlines(True),limited.splitlines(True),
            fromfile='a/libraries/AP_TECS/AP_TECS_Paraglider.cpp',tofile='b/libraries/AP_TECS/AP_TECS_Paraglider.cpp')))
        cpp,header,frontend=governor((PREV/'AP_CustomControl_LQI.cpp.experiment.txt').read_text(),
                                   original[FILES[3]].decode(),original[FILES[2]].decode())
        write(FILES[4],cpp);write(FILES[3],header);write(FILES[2],frontend)
        # Retain a clean governor-only patch separately from the sensor research patch.
        clean=governor(original[FILES[4]].decode(),original[FILES[3]].decode(),original[FILES[2]].decode())
        patch=''
        for p,text in zip([FILES[4],FILES[3],FILES[2]],clean):
            rel=str(p.relative_to(AP));patch+=''.join(difflib.unified_diff(original[p].decode().splitlines(True),text.splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
        (HERE/'governor.patch').write_text(patch)
        train,valid=specs()
        (HERE/'protocol.json').write_text(json.dumps(dict(parameters=PARMS,candidates=CANDIDATES,
            train=train,validation=valid,selection='See score.py: predeclared four-objective loss plus stability/effort gates; select on training only.'),indent=2)+'\n')
        from score import analyze_phase, select
        for name,cfg in CANDIDATES.items():
            design=synthesize(name,cfg);write(FILES[5],(design/'AP_CustomControl_LQI_gains.h').read_bytes())
            with (HERE/f'{name}-build.log').open('w') as log:
                rc=subprocess.run(['bash',str(VIEW/'tools/build/build_sitl_wasm.sh'),str(AP)],cwd=ROOT,env=ENV,
                    stdout=log,stderr=subprocess.STDOUT).returncode
            # Build script writes these artifacts itself; preserve them for restoration checking.
            for p in FILES[7:11]:expected[p]=p.read_bytes()
            if rc:raise RuntimeError('Build failed '+name)
            runtime=design/'runtime';runtime.mkdir()
            for p in FILES:
                (runtime/p.name).write_bytes(p.read_bytes())
            (runtime/'hashes.json').write_text(json.dumps({p.name:sha(p) for p in runtime.iterdir()},indent=2)+'\n')
            if name=='fixed':run_cases('training','standard',train,runtime)
            run_cases('training',name,train,runtime)
            analyze_phase(HERE/'training')
        selection=select(HERE/'training');chosen=selection['selected']
        # No validation result exists before the final training-only selection is saved.
        for name in dict.fromkeys(['standard','fixed',chosen]):
            runtime=HERE/'designs'/('fixed' if name=='standard' else name)/'runtime'
            install_runtime(runtime)
            run_cases('validation',name,valid,runtime)
            run_cases('missions',name,valid,runtime,mission=True)
        analyze_phase(HERE/'validation')
    finally:
        conflicts=[]
        for p,b in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(b);assert p.read_bytes()==b
        for p,src in [(HARNESS,HERE/'case.ts'),(MISSION,HERE/'mission.ts')]:
            if p.exists() and p.read_bytes()==src.read_bytes():p.unlink()
        (HERE/'restoration.json').write_text(json.dumps(dict(conflicts=conflicts,verified=len(FILES)-len(conflicts)),indent=2)+'\n')
        if conflicts:raise RuntimeError('Concurrent files preserved: '+str(conflicts))


if __name__=='__main__':main()
