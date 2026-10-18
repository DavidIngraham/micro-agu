from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'candidate-envelope.json').read_text());fig,axs=plt.subplots(2,3,figsize=(14,8));summary=[]
for di,flight in enumerate(['hood_river','trout_lake']):
 for slack in [0.,6.,12.151707100207533]:
  rs=sorted([r for r in rows if r['flight']==flight and r['slack_mm']==slack and r['throttle']==.4 and r['status']=='finite'],key=lambda r:r['right_mm']-r['left_mm']);x=[r['right_mm']-r['left_mm'] for r in rs]
  for j,k in enumerate(['turn_rate','climb']):axs[di,j].plot(x,[r[k] for r in rs],'.-',label=f'left offset {slack:.2f}mm')
  ns=sorted([r for r in rows if r['flight']==flight and r['slack_mm']==slack and r['left_mm']==0 and r['right_mm']==0 and r['status']=='finite'],key=lambda r:r['throttle']);axs[di,2].plot([r['throttle'] for r in ns],[r['climb'] for r in ns],'.-',label=f'{slack:.2f}mm')
 for j,label in enumerate(['Course rate at throttle0.4 (deg/s)','Climb at throttle0.4 (m/s)','Neutral-brake climb (m/s)']):axs[di,j].set(title=flight+' / '+label,xlabel='Signed commanded brake (right positive), mm' if j<2 else 'Throttle fraction');axs[di,j].grid(alpha=.2);axs[di,j].axhline(0,color='gray',lw=.5)
 summary.append(dict(flight=flight,runs=sum(r['flight']==flight for r in rows),finite=sum(r['flight']==flight and r['status']=='finite' for r in rows),near_steady=sum(r['flight']==flight and r.get('near_steady',False) for r in rows)))
axs[0,0].legend(fontsize=8);fig.suptitle('Candidate airborne envelope:90s constant commands, final20s averages; geometry-derived linear yaw stiffness');fig.tight_layout();fig.savefig(R/'candidate-envelope.png',dpi=160);(R/'candidate-envelope-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
