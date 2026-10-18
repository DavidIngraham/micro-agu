from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'atlas-results.json').read_text())+json.loads((R/'modal-atlas-results.json').read_text());names=['reference','combined_small','modal_common','modal_epoch'];labels=['Starting physics','Aero correction','CAD + gyro modes','CAD + gyro + TL rigging'];fig,ax=plt.subplots(2,4,figsize=(15,8));summary=[]
for di,day in enumerate(['2026-01-03','2026-02-15']):
 for j,(key,label) in enumerate([('course_E_deg_s','Course drift score (deg/s)'),('bank_rmse_deg','Bank RMSE (deg)'),('speed_rmse','Speed RMSE (m/s)'),('climb_rmse','Climb RMSE (m/s)')]):
  values=[]
  for name in names:
   g=[r for r in rows if r['day']==day and r['variant']==name];values.append(np.mean([r[key] for r in g]))
  ax[di,j].bar(range(4),values,color=['.5','C0','C2','C1']);ax[di,j].set_xticks(range(4),labels,rotation=35,ha='right',fontsize=8);ax[di,j].set_title(day);ax[di,j].set_ylabel(label);ax[di,j].grid(axis='y',alpha=.2)
 for name in names:
  g=[r for r in rows if r['day']==day and r['variant']==name];summary.append(dict(day=day,variant=name,episodes=len(g),**{k:float(np.mean([r[k] for r in g])) for k in ['course_E_deg_s','bank_rmse_deg','speed_rmse','climb_rmse','climb_bias']}))
fig.suptitle('A useful tradeoff: better modal mechanics and Trout Lake fit, weaker Hood River fit than aero-only');fig.tight_layout();fig.savefig(R/'candidate-tradeoffs.png',dpi=150);(R/'candidate-tradeoffs.json').write_text(json.dumps(summary,indent=2))
