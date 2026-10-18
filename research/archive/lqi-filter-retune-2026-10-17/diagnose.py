"""Dimensional inspection of the prior crosswind regression; no tuning on new validation."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;BACK=HERE.parent/'lqi-backport-2026-10-17'
rows=[];fig,axes=plt.subplots(4,1,figsize=(10,9),sharex=True,layout='constrained')
for case in [80,81,82,83,84,85,86,87]:
    for variant in ['deployed_off','deployed_on']:
        d=json.loads((BACK/f'validation/{variant}/{case}.json').read_text());ss=d['samples']
        t=np.array([s['time'] for s in ss]);y=np.array([s['truth'] for s in ss]);wind=np.array([s['wind'] for s in ss])
        rot=R.from_quat(y[:,3:7]);att=rot.as_euler('xyz',degrees=True);air=rot.inv().apply(y[:,7:10]-wind)
        u=np.array([s['command'] for s in ss]);brake=np.array([s['brakes'] for s in ss])/(np.array([1300,1500])*np.pi*55/360*.1)
        before=(t>255)&(t<260);during=(t>=260)&(t<280);sample=np.flatnonzero(t>=260)[0]
        sample=np.flatnonzero((t>=260)&(np.linalg.norm(wind,axis=1)>1))[0]
        bodywind=rot[sample].inv().apply(wind[sample])
        rows.append(dict(case=case,variant=variant,heading_before_deg=float(np.median(att[before,2])),
          pitch_before_deg=float(np.median(att[before,1])),speed_before=float(np.median(np.linalg.norm(air[before],axis=1))),
          gust_body_xyz=bodywind.tolist(),peak_throttle=float(max(u[during,0])),
          throttle_floor_fraction=float(np.mean(u[during,0]<.01)),throttle_ceiling_fraction=float(np.mean(u[during,0]>.594)),
          common_brake_before=float(np.median(brake[before].min(axis=1))),common_brake_peak=float(max(brake[during].min(axis=1))),
          pitch_peak_deg=float(max(abs(att[during,1]-np.median(att[before,1]))))))
        if case==83:
            mask=(t>250)&(t<290);label='Filter off' if variant.endswith('off') else '0.7 s filter'
            for ax,v in zip(axes,[att[:,1],u[:,0],brake.min(axis=1),np.linalg.norm(air,axis=1)]):ax.plot(t[mask]-260,v[mask],label=label)
for ax,label in zip(axes,['Payload pitch (°)','Throttle fraction','Common brake fraction','True airspeed (m/s)']):
    ax.set_ylabel(label);ax.axvspan(0,5,color='grey',alpha=.15);ax.grid(alpha=.2)
axes[0].legend();axes[-1].set_xlabel('Seconds from gust onset')
fig.suptitle('Published tune, previously seen case 83: crosswind response\nShading: 2 m/s wind pulse; truth is used only for evaluation')
fig.savefig(HERE/'crosswind-diagnosis.png',dpi=150)
(HERE/'crosswind-diagnosis.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
