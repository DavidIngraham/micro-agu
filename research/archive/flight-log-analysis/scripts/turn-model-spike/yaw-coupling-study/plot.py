from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'results.json').read_text());fig,axs=plt.subplots(2,3,figsize=(13,8))
for i,day in enumerate(['2026-01-03','2026-02-15']):
 for brakes in [[0,0],[20,0],[0,20]]:
  rr=[r for r in rows if r['day']==day and r['brakes_mm']==brakes];k=[r['k_yaw'] for r in rr]
  for j,(key,label) in enumerate([('relative_rotation_z_mean_deg','Relative twist z (deg)'),('turn','Turn rate (deg/s)'),('climb','Climb (m/s)')]):
   yy=[r[key] if j==0 else r['mean_speed_climb_turn_bank'][2 if j==1 else 1] for r in rr];axs[i,j].plot(k,yy,'o-',label=str(brakes)+' mm');axs[i,j].set(xlabel='Yaw stiffness N m/rad',ylabel=label,title=day);axs[i,j].grid(alpha=.2);axs[i,j].legend()
fig.suptitle('Two-body yaw stiffness sensitivity — last10s means, not flight validation');fig.tight_layout();fig.savefig(R/'sensitivity.png',dpi=150)
print(json.dumps([r for r in rows if r['brakes_mm']==[0,0]],indent=2))
