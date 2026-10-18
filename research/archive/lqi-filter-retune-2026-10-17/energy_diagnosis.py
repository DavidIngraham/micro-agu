"""Training-only steady-window speed/brake inspection; HUD EAS is estimated without a pitot."""
import json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
rows=[]
for path in (HERE/'training').glob('*/*.json'):
    if not path.stem.isdigit():continue
    d=json.loads(path.read_text())
    if not d['complete']:continue
    ss=[s for s in d['samples'] if 170<s['time']<210]
    if not ss:continue
    y=np.array([s['truth'] for s in ss]);w=np.array([s['wind'] for s in ss])
    brakes=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
    rows.append(dict(variant=d['spec']['variant'],case=d['spec']['id'],
        true_airspeed=float(np.mean(np.linalg.norm(y[:,7:10]-w,axis=1))),
        hud_estimated_EAS=float(np.mean([s['pitotEas'] for s in ss])),
        common_brake=float(np.mean(brakes.min(axis=1))),throttle=float(np.mean([s['command'][0] for s in ss]))))
(HERE/'energy-diagnosis.json').write_text(json.dumps(rows,indent=2))
for name in sorted({r['variant'] for r in rows}):
    rr=[r for r in rows if r['variant']==name]
    print(name,{k:round(float(np.median([r[k] for r in rr])),4) for k in ['true_airspeed','hud_estimated_EAS','common_brake','throttle']})
