"""Ten-minute CRUISE checks catch slow brake/speed-integrator drift."""
import json
import campaign as c
HERE=c.HERE;BACK=c.BACK;b=c.b
def main():
    selected=json.loads((HERE/'selection.json').read_text())['selected'];assert selected
    src=(HERE/'mission.ts').read_text();assert src.count('start + 180')==1
    harness=src.replace('start + 180','start + 600')
    files=b.FILES[7:10];original={p:p.read_bytes() for p in files};expected=dict(original)
    def write(p,data):p.write_bytes(data);expected[p]=data
    assert not b.MISSION.exists();b.MISSION.write_text(harness)
    try:
        chosen=BACK/'runtime/deployed' if selected=='filtered' else HERE/'designs'/selected/'runtime'
        for name,runtime in [('published',BACK/'runtime/deployed'),(selected,chosen)]:
            for p in files:write(p,(runtime/p.name).read_bytes())
            cases=[s for s in json.loads((HERE/'protocol.json').read_text())['validation'] if s['id'] in [101,105,109]]
            for s in cases:s['parameters']=dict(CP3_HRTC=0 if name=='published' else .7,ARSPD_TYPE=0,ARSPD_USE=0)
            b.run_cases('endurance',name,cases,runtime,mission=True)
    finally:
        conflicts=[]
        for p,data in original.items():
            if p.read_bytes()!=expected[p]:conflicts.append(str(p));continue
            p.write_bytes(data)
        if b.MISSION.read_text()==harness:b.MISSION.unlink()
        (HERE/'endurance-restoration.json').write_text(json.dumps(dict(conflicts=conflicts),indent=2));assert not conflicts
if __name__=='__main__':main()
