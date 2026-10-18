from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'roll-coupling-results.json').read_text());fig,axs=plt.subplots(2,3,figsize=(14,8))
for i,day in enumerate(['2026-01-03','2026-02-15']):
 for brakes in [[0,0],[20,0],[0,20]]:
  rr=[r for r in rows if r['day']==day and r['brakes_mm']==brakes and r['torque']=='APC' and r['status']=='finite'];x=[r['k_roll'] for r in rr]
  for j,key in enumerate(['payload_bank','canopy_bank','turn']):
   axs[i,j].plot(x,[r[key] if key!='turn' else r['mean_speed_climb_turn_bank'][2] for r in rr],'o-',label=str(brakes)+'mm');axs[i,j].set(xlabel='Roll stiffness N m/rad',ylabel='deg' if j<2 else 'deg/s',title=day+' '+key);axs[i,j].grid(alpha=.2);axs[i,j].legend()
fig.tight_layout();fig.savefig(R/'roll-coupling.png',dpi=150)
print(json.dumps([{k:r[k] for k in ['day','k_roll','payload_bank','canopy_bank','mean_speed_climb_turn_bank']} for r in rows if r['brakes_mm']==[0,0] and r['torque']=='APC' and r['status']=='finite'],indent=2))
