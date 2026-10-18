"""AI-assisted isolated turn envelope build; preserve the user's research tree."""
import difflib
import hashlib
import json
import os
from pathlib import Path
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]; AP=ROOT.parent/'ardupilot-browser-9dof'; VIEW=ROOT/'viewer'
BASE='010ada6a4f0b26910fc18ecca176d2e65c5ed9d7'
SOURCE=HERE.parent/'lqi-filter-retune-2026-10-17/designs/responsive_softspeed/runtime'
ENV=dict(os.environ,PATH='/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:/usr/local/bin:/usr/bin:/bin')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    changed=subprocess.check_output(['git','diff','--name-only','HEAD'],cwd=AP,text=True).splitlines()
    src=[AP/'libraries/AP_CustomControl'/f for f in ['AP_CustomControl.cpp','AP_CustomControl_LQI.h','AP_CustomControl_LQI.cpp','AP_CustomControl_LQI_gains.h']]
    outputs=[VIEW/'generated/ardupilot'/f for f in ['arduplane.js','arduplane.wasm']]+[VIEW/'assets/runtime'/f for f in ['build.json','RUST-NOTICES.txt']]
    paths=list(dict.fromkeys(src+[AP/p for p in changed if not p.startswith('modules/')]+outputs))
    original={p:p.read_bytes() for p in paths}; expected=dict(original)
    backup=HERE/'original';backup.mkdir(exist_ok=True);manifest=[]
    for i,(p,data) in enumerate(original.items()):
        dest=backup/str(i)
        if dest.exists(): assert dest.read_bytes()==data
        dest.write_bytes(data);manifest.append(dict(path=str(p),backup=str(i),sha256=sha(p)))
    (backup/'manifest.json').write_text(json.dumps(manifest,indent=2))
    def write(p,data):
        data=data.encode() if isinstance(data,str) else data;p.write_bytes(data);expected[p]=data
    try:
        for p in paths:
            if p.is_relative_to(AP):write(p,subprocess.check_output(['git','show',BASE+':'+str(p.relative_to(AP))],cwd=AP))
        for p in src:write(p,(SOURCE/p.name).read_bytes())
        cpp=src[2].read_text();hdr=src[1].read_text()
        hdr=hdr.replace('    AP_Float _height_rate_tc;','    AP_Float _height_rate_tc;\n    AP_Float _turn_max;')
        assert '_turn_max' in hdr
        cpp=cpp.replace('    AP_GROUPEND','''    // @Param: RMAX
    // @DisplayName: LQI heading rate limit
    // @Description: Maximum heading rate reference. Values above 10 deg/s are experimental until validated for the aircraft model.
    // @Units: deg/s
    // @Range: 1 60
    // @User: Advanced
    AP_GROUPINFO("RMAX", 5, AP_CustomControl_LQI, _turn_max, 10.0f),
    AP_GROUPEND''')
        cpp=cpp.replace('return isfinite(_height_rate_tc.get())','return isfinite(_turn_max.get()) && _turn_max >= 1 && _turn_max <= 60 &&\n        isfinite(_height_rate_tc.get())')
        old='constrain_float(in.reference.heading_rate_rads, -radians(10), radians(10))'
        assert cpp.count(old)==1
        cpp=cpp.replace(old,'constrain_float(in.reference.heading_rate_rads, -radians(_turn_max), radians(_turn_max))')
        # Research-only instrumentation; not part of a deployment patch.
        production=cpp
        cpp='#include <stdio.h>\n'+cpp
        cpp=cpp.replace('    demand = _held;\n    return demand.finite();','''    // AI-assisted research telemetry, sampled at 10 Hz. No feedback changes.
    static uint8_t diagnostic_tick;
    if (++diagnostic_tick >= 5) {
        diagnostic_tick = 0;
        printf("TURNID,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\\n",
            double(degrees(in.reference.heading_rate_rads)), double(degrees(_turn_reference)),
            double(degrees(heading_rate)), double(in.state.true_airspeed_mps), double(speed_target),
            double(_requested[0]), double(_requested[1]), double(_requested[2]),
            double(_applied[0]), double(_applied[1]), double(_applied[2]),
            double(_integral[0]), double(_integral[1]), double(_integral[2]), double(_turn_max.get()));
    }
    demand = _held;
    return demand.finite();''')
        write(src[1],hdr);write(src[2],cpp)
        runtime=HERE/'runtime';runtime.mkdir(exist_ok=True)
        (runtime/'production-LQI.cpp').write_text(production)
        with (HERE/'build.log').open('w') as log:
            rc=subprocess.run(['bash',str(VIEW/'tools/build/build_sitl_wasm.sh'),str(AP)],cwd=ROOT,env=ENV,stdout=log,stderr=subprocess.STDOUT).returncode
        for p in outputs:expected[p]=p.read_bytes()
        if rc:raise RuntimeError('Build failed; see build.log')
        for p in src+outputs:(runtime/p.name).write_bytes(p.read_bytes())
        patch=''
        for p in src:
            rel=str(p.relative_to(AP));before=subprocess.check_output(['git','show',BASE+':'+rel],cwd=AP,text=True)
            patch+=''.join(difflib.unified_diff(before.splitlines(True),p.read_text().splitlines(True),fromfile='a/'+rel,tofile='b/'+rel))
        (runtime/'source.patch').write_text(patch)
        (runtime/'hashes.json').write_text(json.dumps({p.name:sha(p) for p in runtime.iterdir()},indent=2))
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        (HERE/'restoration.json').write_text(json.dumps(dict(conflicts=conflicts,files=len(original)),indent=2))
        assert not conflicts,conflicts
if __name__=='__main__':main()
