from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
rows=json.loads((R/'preload-atlas-results.json').read_text())+json.loads((R/'preload-hr-extension-results.json').read_text());base=json.loads((R/'candidate-v2-atlas-results.json').read_text());rows+=[dict(r,variant='preload_0.0') for r in base];selected={(r['day'],r['start']) for r in json.loads((R/'preload-calibration-windows.json').read_text())['windows']};out=[]
for day in ['2026-01-03','2026-02-15']:
 for preload in ([0.,1.,2.,4.,6.,8.] if day=='2026-01-03' else [0.,1.,2.,4.]):
  for split in ['calibration','remaining','all']:
   a=[r for r in rows if r['day']==day and r['variant']==f'preload_{preload}' and (split=='all' or (((day,r['start']) in selected)==(split=='calibration')))];out.append(dict(day=day,preload=preload,split=split,n=len(a),**{k:float(np.mean([r[k] for r in a])) for k in ['course_E_deg_s','bank_rmse_deg','speed_rmse','climb_rmse']}))
(R/'preload-summary.json').write_text(json.dumps(out,indent=2));print(json.dumps([r for r in out if r['split']=='all'],indent=2))
fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,day in enumerate(['2026-01-03','2026-02-15']):
 for split in ['calibration','remaining']:
  a=[r for r in out if r['day']==day and r['split']==split]
  for j,key in enumerate(['course_E_deg_s','bank_rmse_deg']):axs[i,j].plot([r['preload'] for r in a],[r[key] for r in a],'o-',label=split);axs[i,j].set(title=f'{day}: {key}',xlabel='Assumed right-brake preload (mm)');axs[i,j].grid(alpha=.2);axs[i,j].legend()
fig.suptitle('One constant brake-preload hypothesis per flight; all mechanics held fixed');fig.tight_layout();fig.savefig(R/'preload-tradeoffs.png',dpi=150)

