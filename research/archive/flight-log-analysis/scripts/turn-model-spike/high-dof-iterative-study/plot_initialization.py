from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'initialization-sensitivity-results.json').read_text());fig,axs=plt.subplots(1,2,figsize=(12,5));summary=[]
for i,day in enumerate(['2026-01-03','2026-02-15']):
 starts=sorted(set(r['start'] for r in rows if r['day']==day))
 for model,color,shift in [('reference','#999999',-.07),('retained','#228866',.07)]:
  for j,start in enumerate(starts):
   a=[r for r in rows if r['day']==day and r['start']==start and r['model']==model];v=[r['course_E_deg_s'] for r in a];n=next(r['course_E_deg_s'] for r in a if r['initialization']=='nominal');axs[i].errorbar(j+shift,n,yerr=[[n-min(v)],[max(v)-n]],fmt='o',color=color,capsize=4,label=model if j==0 else None);summary.append(dict(day=day,start=start,model=model,nominal=n,min=min(v),max=max(v)))
 axs[i].set_xticks(range(len(starts)),[str(s)+'s' for s in starts]);axs[i].set(title='Hood River' if i==0 else 'Trout Lake',ylabel='Course error (deg/s)',xlabel='Flight log window');axs[i].legend();axs[i].grid(alpha=.2)
fig.suptitle('Sensitivity to unmeasured initial canopy attitude/rate; ranges are hypotheses, not confidence intervals');fig.tight_layout();fig.savefig(R/'initialization-sensitivity.png',dpi=150);(R/'initialization-sensitivity-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
