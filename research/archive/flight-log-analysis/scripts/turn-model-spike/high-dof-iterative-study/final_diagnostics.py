from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
rows=json.loads((R/'soft-v3-atlas-results.json').read_text())
for day in ['2026-01-03','2026-02-15']:
 for variant in ['soft_v3_raw','soft_v3_retrim']:
  q=[x for x in rows if x['day']==day and x['variant']==variant];print(day,variant,{k:float(np.mean([x[k] for x in q])) for k in ['course_E_deg_s','bank_rmse_deg','speed_rmse','climb_rmse']})
results=json.loads((R/'v3-robust-results.json').read_text());fig,axs=plt.subplots(1,3,figsize=(15,5));cases=list(dict.fromkeys(x['case'] for x in results));x=np.arange(len(cases))
for flight,shift,color in [('hood_river',-.18,'#d88735'),('trout_lake',.18,'#197d9e')]:
 q=[next(q for q in results if q['flight']==flight and q['case']==case) for case in cases]
 for row in q:
  d=np.load(R/f"v3-robust-{flight}-{row['case']}-power_trim.npz")['data'];u=d[:,4];offset=12.151707100207533 if flight=='hood_river' else 0.;left=np.maximum(-u,0)*72+(u<0)*offset;right=np.maximum(u,0)*72;row['actuator_clipping_fraction']=float(np.mean((left>62.4+1e-9)|(right>72+1e-9)))
 for ax,key in zip(axs,['course_rate_rmse','climb_rmse','actuator_clipping_fraction']):ax.bar(x+shift,[r[key] for r in q],.36,label=flight.replace('_',' '),color=color)
for ax,ylabel in zip(axs,['Course-rate RMSE (deg/s)','Climb RMSE (m/s)','Physical actuator clipping fraction']):ax.set_xticks(x,cases,rotation=45,ha='right');ax.set_ylabel(ylabel);ax.grid(axis='y',alpha=.2)
axs[0].legend();axs[2].set_ylim(0,1);fig.suptitle('One fixed controller across eight model variants; ideal sensors, model-derived feedforward');fig.tight_layout();fig.savefig(R/'controller-robustness.png',dpi=150);(R/'v3-robust-results.json').write_text(json.dumps(results,indent=2));print('Physical clipping:',[(r['flight'],r['case'],r['actuator_clipping_fraction']) for r in results])
