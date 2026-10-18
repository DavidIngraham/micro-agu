"""Locate paired peak regressions; descriptive, not a new acceptance gate."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from build import HERE
rows=[]
for case in [100,101,102,104,108,109]:
    cohort='validation' if case<106 else 'release_validation'
    for variant in ['reference','map_aw_observer' if case<106 else 'candidate']:
        d=json.loads((HERE/cohort/variant/'lqi-25'/f'{case:03}.json').read_text());ss=[s for s in d['samples'] if s['time']<200];x=np.array([s['truth'] for s in ss]);rot=Rotation.from_quat(x[:,3:7]);pitch=np.rad2deg(rot.inv().apply(x[:,23:26]-x[:,10:13])[:,1]);i=int(np.argmax(abs(pitch)));s=ss[i];near=[a for a in ss if abs(a['time']-s['time'])<2]
        rows.append(dict(case=case,variant=variant,time=s['time'],phase=s['phase'],relative_pitch_rate=float(pitch[i]),wind=s['wind'],command=s['command'],diagnostic=s['diagnostic'],local_altitude_range=[min(-a['truth'][2] for a in near),max(-a['truth'][2] for a in near)],local_pitch_range_deg=[float(np.rad2deg(min(a['pitch'] for a in near))),float(np.rad2deg(max(a['pitch'] for a in near)))]))
(HERE/'deployment/pitch-peak-diagnosis.json').write_text(json.dumps(rows,indent=2))
for r in rows:print(r['case'],r['variant'],round(r['time'],1),r['phase'],round(r['relative_pitch_rate'],2),'alt',r['local_altitude_range'],'pitch',r['local_pitch_range_deg'])
