"""Evaluate the frozen inclusive selection if it differs from gain-only selection."""
import json
from pathlib import Path
import importlib.util
HERE=Path(__file__).resolve().parent;BACK=HERE.parent/'lqi-backport-2026-10-17'
spec=importlib.util.spec_from_file_location('campaign_helpers',HERE/'campaign.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c);b=c.b
def main():
    assert not json.loads((HERE/'restoration.json').read_text())['conflicts']
    for r in json.loads((HERE/'original/manifest.json').read_text()):assert b.sha(Path(r['path']))==r['sha256']
    primary=(HERE/'selection.json').read_bytes()
    frozen=HERE/('selection-with-speed-task.json' if (HERE/'selection-with-speed-task.json').exists() else 'selection-with-filtered.json')
    decision=json.loads(frozen.read_text())
    chosen=decision['selected'];(HERE/'gain-only-selection.json').write_bytes(primary)
    files=b.FILES[7:10];original={p:p.read_bytes() for p in files};expected=dict(original)
    def write(p,data):p.write_bytes(data);expected[p]=data
    try:
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:assert not p.exists();p.write_bytes((HERE/name).read_bytes())
        if chosen:
            selected_runtime=BACK/'runtime/deployed' if chosen=='filtered' else HERE/'designs'/chosen/'runtime'
            for name,runtime in [('published',BACK/'runtime/deployed'),('local',BACK/'runtime/local'),(chosen,selected_runtime)]:
                for phase in ['validation','missions']:
                    if (HERE/phase/name/'runs.json').exists():continue
                    for p in files:write(p,(runtime/p.name).read_bytes())
                    cases=json.loads((HERE/'protocol.json').read_text())['validation']
                    for s in cases:s['parameters']=dict(CP3_HRTC=0 if name=='published' else .7,ARSPD_TYPE=0,ARSPD_USE=0)
                    b.run_cases(phase,name,cases,runtime,mission=phase=='missions')
        (HERE/'selection.json').write_bytes(frozen.read_bytes())
        (HERE/'selection-with-filtered.json').write_bytes(frozen.read_bytes())
        c.report()
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        for p,name in [(b.HARNESS,'case.ts'),(b.MISSION,'mission.ts')]:
            if p.exists() and p.read_bytes()==(HERE/name).read_bytes():p.unlink()
        (HERE/'selection-restoration.json').write_text(json.dumps(dict(conflicts=conflicts),indent=2));assert not conflicts
if __name__=='__main__':main()
