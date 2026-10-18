import os

from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(os.environ.get('PG_RESEARCH_ROOT', '/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/l1-regression-2026-10-13'))
d=json.loads((r/'mission-results.json').read_text())
profiles=['standard-calm','standard-crosswind','standard-gust','tight-calm','wide-crosswind']
labels=['25 m, calm','25 m, crosswind','25 m, gusts','20 m, calm','45 m, crosswind']
fig,axs=plt.subplots(2,2,figsize=(12,8))
for period,color in [(8,'#d67b42'),(10,'#168b9a')]:
 for ax,metric in zip(axs.flat,['max_corner_overrun_m','max_waypoint_height_error_m','radial_rmse_m','command_travel_per_s']):
  vals=[]
  for profile in profiles:
   z=d.get(profile+'-'+str(period))
   vals.append(np.nan if z is None else (max(x[metric] for x in z['loiters'].values()) if metric in ['radial_rmse_m','command_travel_per_s'] else z[metric]))
  ax.plot(range(len(profiles)),vals,'o-',color=color,label=f'{period} s period')
for ax,title in zip(axs.flat,['Maximum corner overrun (m)','Maximum waypoint height error (m)','Worse-direction loiter radial RMS (m)','Worse-direction loiter brake variation / s']):
 ax.set_title(title);ax.set_xticks(range(len(profiles)),labels,rotation=20,ha='right');ax.grid(alpha=.2);ax.legend()
fig.tight_layout();fig.savefig(r/'mission-comparison.png',dpi=150)
if (r/'landing-results.json').exists():
 d=json.loads((r/'landing-results.json').read_text());fig,axs=plt.subplots(1,3,figsize=(13,4))
 for prefix,label,color in [('validation','8 s','#d67b42'),('landing10','10 s','#168b9a')]:
  for ax,key in zip(axs,['roll_90pct_span','rate_error_rms','miss_m']):
   ax.plot(range(12),[d.get(f'{prefix}-{i:02d}',{}).get(key,np.nan) for i in range(12)],'o-',label=label,color=color)
 for ax,title in zip(axs,['Pre-flare roll span (deg, central 90%)','Truth rate tracking RMSE (deg/s)','Ground-crossing miss (m)']):
  ax.set_title(title);ax.set_xlabel('Regression condition');ax.grid(alpha=.2);ax.legend()
 fig.tight_layout();fig.savefig(r/'landing-comparison.png',dpi=150)
