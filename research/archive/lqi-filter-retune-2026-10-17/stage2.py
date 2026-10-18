"""Second training-only gain pass; no hold-out result exists before selection."""
import difflib,json,subprocess
from pathlib import Path
import campaign as c
import synthesize as s
from evaluate import select,report
HERE=c.HERE;BACK=c.BACK;b=c.b
CANDIDATES={
 'responsive_brake3k':dict(pitch=90,height=4,integral=12,throttle=500,common=3000),
 'responsive_brake10k':dict(pitch=90,height=4,integral=12,throttle=500,common=10000),
 'efficient_brake10k':dict(pitch=90,height=2,integral=6,throttle=700,common=10000),
 'responsive_softspeed':dict(pitch=90,height=4,integral=12,throttle=500,common=3000,speed=.3,speed_integral=.3),
}
def main():
    assert json.loads((HERE/'selection.json').read_text())['selected'] is None
    assert not any(p.stem.isdigit() for p in (HERE/'validation').glob('*/*.json'))
    manifest=json.loads((HERE/'original/manifest.json').read_text());files=[Path(r['path']) for r in manifest]
    for r in manifest:assert b.sha(Path(r['path']))==r['sha256']
    for name in ['selection.json','selection-with-filtered.json','acceptance.json']:(HERE/('stage1-'+name)).write_bytes((HERE/name).read_bytes())
    (HERE/'stage2-protocol.json').write_text(json.dumps(dict(candidates=CANDIDATES,
       rationale='First pass improved tracking but failed energy. Increase common effort; one candidate relaxes speed feedback. Keep speed target, observer and reference maps fixed.',
       extra_gate='Each climb/descent mean true speed must stay within 10% of published in every matched case. No prior release gate relaxed.'),indent=2))
    s.CANDIDATES=CANDIDATES;s.main()
    original={p:p.read_bytes() for p in files};expected=dict(original)
    def write(p,data):
        data=data.encode() if isinstance(data,str) else data;p.write_bytes(data);expected[p]=data
    def run(phase,name,cases,runtime):
        for p in b.FILES[7:10]:write(p,(runtime/p.name).read_bytes())
        ss=json.loads(json.dumps(cases))
        for x in ss:x['parameters']=dict(CP3_HRTC=0 if name=='published' else .7,ARSPD_TYPE=0,ARSPD_USE=0)
        b.run_cases(phase,name,ss,runtime,mission=phase=='missions')
    protocol=json.loads((HERE/'protocol.json').read_text())
    try:
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:assert not p.exists();p.write_bytes((HERE/name).read_bytes())
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
            run('training',name,protocol['training'],runtime)
        chosen=select();(HERE/'selection-with-filtered.json').write_bytes((HERE/'selection.json').read_bytes())
        if chosen:
            runtime=BACK/'runtime/deployed' if chosen=='filtered' else HERE/'designs'/chosen/'runtime'
            for name,r in [('published',BACK/'runtime/deployed'),('local',BACK/'runtime/local'),(chosen,runtime)]:
                run('validation',name,protocol['validation'],r);run('missions',name,protocol['validation'],r)
        report()
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:
            if p.exists() and p.read_bytes()==(HERE/name).read_bytes():p.unlink()
        (HERE/'stage2-restoration.json').write_text(json.dumps(dict(conflicts=conflicts,verified=len(original)-len(conflicts)),indent=2));assert not conflicts
if __name__=='__main__':main()
