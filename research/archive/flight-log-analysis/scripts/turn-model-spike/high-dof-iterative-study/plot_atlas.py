from pathlib import Path
import json,numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'atlas-results.json').read_text());summary=[];fig,ax=plt.subplots(2,3,figsize=(14,8))
for di,day in enumerate(['2026-01-03','2026-02-15']):
 base=sorted([r for r in rows if r['day']==day and r['variant']=='reference'],key=lambda r:r['start']);starts=[r['start'] for r in base]
 for name in ['reference','combined_small','arc075','arc060']:
  rs=sorted([r for r in rows if r['day']==day and r['variant']==name],key=lambda r:r['start']);good=[r for r in rs if r['status']=='finite'];s=dict(day=day,variant=name,count=len(rs),finite=len(good))
  for k in ['course_E_deg_s','bank_rmse_deg','speed_rmse','climb_rmse']:s[k]=float(np.mean([r[k] for r in good]))
  for k in ['speed_bias','climb_bias']:s[k+'_rms']=float(np.sqrt(np.mean([r[k]**2 for r in good])))
  if name=='combined_small':s['course_improved_episodes']=sum(r['course_E_deg_s']<b['course_E_deg_s'] for r,b in zip(rs,base));s['largest_course_regressions']=sorted([dict(start=r['start'],delta=r['course_E_deg_s']-b['course_E_deg_s']) for r,b in zip(rs,base)],key=lambda r:r['delta'],reverse=True)[:5]
  summary.append(s)
  for j,k in enumerate(['course_E_deg_s','bank_rmse_deg','climb_rmse']):ax[di,j].plot([r['start'] for r in good],[r[k] for r in good],'.-',label=name,lw=1,ms=3)
 for j,label in enumerate(['Course drift score (deg/s equivalent)','Bank RMS error (deg)','Climb RMS error (m/s)']):ax[di,j].set(title=day+' / '+label,xlabel='Flight log time (s)');ax[di,j].grid(alpha=.2)
ax[0,0].legend(fontsize=8);fig.suptitle('All 98 previously selected ordinary 30-second flight windows; lower error is better');fig.tight_layout();fig.savefig(R/'atlas-comparison.png',dpi=160);(R/'atlas-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
