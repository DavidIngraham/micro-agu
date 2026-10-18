"""AI-assisted transition/observer study. Reversible, archived builds, no deployment."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'lqi-balanced-2026-10-17'
spec=importlib.util.spec_from_file_location('prior_campaign',OLD/'campaign.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
b.HERE=HERE
b.HARNESS=b.VIEW/'tests/integration/transitions-case.ts'
b.MISSION=b.VIEW/'tests/integration/transitions-mission.ts'
BASE=dict(pitch=90,height=4,integral=12,throttle=500,vertical=0)
CANDIDATES={
    'fixed':dict(BASE,tc=0,process=1e-5),
    'smooth03':dict(BASE,tc=.3,process=1e-5),
    'smooth07':dict(BASE,tc=.7,process=1e-5),
    'smooth12':dict(BASE,tc=1.2,process=1e-5),
    'smooth07_fastobs':dict(BASE,tc=.7,process=1e-4),
    'smooth07_damped':dict(BASE,pitch=180,tc=.7,process=1e-5),
}

def observer_header(design,process):
    from scipy.linalg import solve_discrete_are
    z=np.load(design/'design.npz');idx=list(z['measured'])+[2,3,4]
    h=np.eye(16)[idx]
    noise=np.diag((np.r_[[np.deg2rad(1)]*2,[.2]*3,[np.deg2rad(1)]*3,.3,[np.deg2rad(.5)]*3]/z['XS'][idx])**2)
    p=solve_discrete_are(z['A'].T,h.T,np.eye(16)*process,noise)
    gain=np.linalg.solve(h@p@h.T+noise,h@p).T
    import re
    def value(a):return '{'+', '.join(value(v) for v in a)+'}' if np.ndim(a) else f'{float(a):.9e}f'
    header=(design/'AP_CustomControl_LQI_gains.h').read_text()
    header,n=re.subn(r'(static constexpr float L_PROP\[16\]\[12\] = )[^;]+;',lambda m:m[1]+value(gain)+';',header)
    assert n==1
    return header

def main():
    backup=HERE/'original';backup.mkdir()
    assert not b.HARNESS.exists() and not b.MISSION.exists()
    original={p:p.read_bytes() for p in b.FILES};expected=dict(original)
    manifest=[]
    for i,(p,data) in enumerate(original.items()):
        (backup/str(i)).write_bytes(data);manifest.append(dict(path=str(p),backup=str(i),sha256=b.sha(p)))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    def write(p,data):
        data=data.encode() if isinstance(data,str) else data;p.write_bytes(data);expected[p]=data
    def runtime_install(runtime):
        for p in b.FILES[7:10]:write(p,(runtime/p.name).read_bytes())
    try:
        for target,src in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:target.write_bytes((HERE/src).read_bytes())
        for p in b.FILES[:2]:write(p,(b.PREV/(p.name+'.experiment.txt')).read_bytes())
        # Six optional vector channels carry the complete normalized state chart.
        h=b.FILES[0].read_text().replace('keytable[39]','keytable[45]')
        h=b.replace(h,'    static Quaternion research_relative;', '    static Vector3f research_truth[6];\n    static Quaternion research_relative;')
        anchor='        { "", "microagu_mode", &research_mode, DATA_FLOAT, false },'
        h=b.replace(h,anchor,anchor+'\n'+''.join(f'        {{ "", "microagu_truth{i}", &research_truth[{i}], DATA_VECTOR3F, false }},\n' for i in range(6)))
        write(b.FILES[0],h)
        c=b.FILES[1].read_text();c=b.replace(c,'Quaternion JSON::research_relative;','Vector3f JSON::research_truth[6];\nQuaternion JSON::research_relative;')
        c=b.replace(c,'    const uint64_t research_mask =', '    if (unsigned(research_mode) == 4 && (received_bitmask & (63ULL << 39)) != (63ULL << 39)) { research_mode = 0; }\n    const uint64_t research_mask =')
        write(b.FILES[1],c)
        write(b.FILES[6],(b.PREV/'physics.ts.experiment.txt').read_bytes())
        write(b.FILES[-1],b.replace(original[b.FILES[-1]].decode(),'const float dn_step_max = MAX(_sink_rate_limit, 0.0f) * _DT;','const float dn_step_max = MIN(MAX(_sink_rate_limit, 0.0f), 1.0f) * _DT;'))
        cpp,header,frontend=b.governor((b.PREV/'AP_CustomControl_LQI.cpp.experiment.txt').read_text(),original[b.FILES[3]].decode(),original[b.FILES[2]].decode())
        header=b.replace(header,'    AP_Float _height_tc;','    AP_Float _height_tc;\n    float _smoothed_climb;')
        cpp=b.replace(cpp,'    _height_reference = 0;','    _height_reference = 0;\n    _smoothed_climb = 0;')
        cpp=b.replace(cpp,'    const float governed_climb = constrain_float(climb, -_height_down.get(), _height_up.get());', '''    const float requested_climb = constrain_float(climb, -_height_down.get(), _height_up.get());
    _smoothed_climb += DT/(RESEARCH_REFERENCE_TC+DT)*(requested_climb-_smoothed_climb);
    const float governed_climb = _smoothed_climb;''')
        cpp=b.replace(cpp,'    const float climb = in.reference.vertical', '''#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && AP_SIM_JSON_ENABLED
    // Research-only full chart truth. Navigation and reference generation retain EKF.
    if (proprio_mode == 4 && in.time_ms-SITL::JSON::research_received_ms < 100) {
        for (uint8_t j = 0; j < 16; j++) {
            _state[j] = SITL::JSON::research_truth[j/3][j%3];
        }
        _state[15] -= _height_origin/XS[15];
    }
#endif
    const float climb = in.reference.vertical''')
        write(b.FILES[4],cpp);write(b.FILES[3],header);write(b.FILES[2],frontend)
        train,_=b.specs()
        rng=np.random.default_rng(2026101709);valid=[]
        for i in range(60,68):
            s=json.loads(json.dumps(train[0]));s['id']=i
            s['scales']={k:float(rng.uniform(.7,1.3)) for k in ['Cmq','joint_c_pitch','joint_c_roll','joint_c_yaw','CLbrake','CDbrake']}
            s['brakeRateScale']=float(rng.uniform(.7,1.1))
            s['wind']=float(rng.uniform(0,2));s['from']=float(rng.uniform(0,360))
            s['weather'].update(turbulence=float(rng.uniform(0,.6)),thermalStrength=float(rng.uniform(0,2)),seed=int(rng.integers(1,2**32-1)))
            valid.append(s)
        (HERE/'protocol.json').write_text(json.dumps(dict(candidates=CANDIDATES,train=train,validation=valid,
            score='score.py; command peaks explicitly included; selection training only',
            sensing='All candidates: 50Hz relative angle, 0.5deg noise, 40ms delay; pitot 2Pa. Ablation separate.'),indent=2))
        from score import analyze_phase,select
        for name,cfg in CANDIDATES.items():
            design=b.synthesize(name,cfg)
            gains=observer_header(design,cfg['process'])+f'\nstatic constexpr float RESEARCH_REFERENCE_TC = {cfg["tc"]}f;\n'.replace('= 0f','= 0.0f')
            (design/'AP_CustomControl_LQI_gains.h').write_text(gains)
            write(b.FILES[5],gains)
            z=np.load(design/'design.npz');s=z['state'];rp=Rotation.from_quat(s[3:7]);rc=Rotation.from_quat(s[16:20])
            trim=dict(euler=rp.as_euler('xyz').tolist(),relative=(rp.inv()*rc).as_quat().tolist(),velocity=rp.inv().apply(s[7:10]).tolist(),payloadRates=rp.inv().apply(s[10:13]).tolist(),canopyRates=rc.inv().apply(s[23:26]).tolist(),rpm=float(z['rpm']),scale=z['XS'].tolist())
            (HERE/'trim.json').write_text(json.dumps(trim))
            with (HERE/f'{name}-build.log').open('w') as log:
                result=subprocess.run(['bash',str(b.VIEW/'tools/build/build_sitl_wasm.sh'),str(b.AP)],cwd=b.ROOT,env=b.ENV,stdout=log,stderr=subprocess.STDOUT)
            for p in b.FILES[7:11]:expected[p]=p.read_bytes()
            if result.returncode:raise RuntimeError('build '+name)
            runtime=design/'runtime';runtime.mkdir()
            for p in b.FILES:(runtime/p.name).write_bytes(p.read_bytes())
            (runtime/'hashes.json').write_text(json.dumps({p.name:b.sha(p) for p in runtime.iterdir()},indent=2))
            if name=='fixed':b.run_cases('training','standard',train,runtime)
            b.run_cases('training',name,train,runtime)
            analyze_phase(HERE/'training')
        chosen=select(HERE/'training')['selected']
        for name in dict.fromkeys(['standard','fixed',chosen]):
            runtime=HERE/'designs'/('fixed' if name=='standard' else name)/'runtime';runtime_install(runtime)
            b.run_cases('validation',name,valid,runtime)
            b.run_cases('missions',name,valid,runtime,mission=True)
        # Ablations diagnose estimation versus control, never select gains.
        for name in dict.fromkeys(['fixed',chosen]):
            runtime=HERE/'designs'/name/'runtime';runtime_install(runtime)
            for sensor in ['ideal','nodelay','stress','truth']:
                cases=json.loads(json.dumps(train))
                for s in cases:s['sensor']=sensor
                b.run_cases('ablation',name+'_'+sensor,cases,runtime)
        analyze_phase(HERE/'validation');analyze_phase(HERE/'ablation')
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        for target,src in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:
            if target.exists() and target.read_bytes()==(HERE/src).read_bytes():target.unlink()
        (HERE/'restoration.json').write_text(json.dumps(dict(conflicts=conflicts,verified=len(original)-len(conflicts)),indent=2))
        if conflicts:raise RuntimeError(conflicts)

if __name__=='__main__':main()
