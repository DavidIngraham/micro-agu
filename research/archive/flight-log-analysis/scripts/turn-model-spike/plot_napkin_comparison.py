from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
O=Path(__file__).parent
fig,axes=plt.subplots(3,2,figsize=(12,10),constrained_layout=True)
rows=json.loads((O/'napkin-results.json').read_text());base=json.loads((O/'baseline-before-napkin/bounded-results.json').read_text());summary=[]
keys=['position_rmse_m','course_rmse_deg','bank_rmse_deg','speed_rmse_m_s','climb_rmse_m_s'];floors=[1,5,1,.1,.1]
for col,day in enumerate(['2026-01-03','2026-02-15']):
 b=min([r for r in base if r['day']==day and r['case']=='nominal' and r['lead_s']==5],key=lambda r:r['cost'])
 cands=[r for r in rows if r['day']==day and r['case']=='nominal']
 if len(cands)<3:continue
 c=min(cands,key=lambda r:r['cost']);ratios=[c['forecast'][k]/max(b['forecast'][k],floor) for k,floor in zip(keys,floors)];summary.append(dict(day=day,baseline=b['forecast'],candidate=c['forecast'],ratios=ratios,score=float(np.mean(ratios))))
 for prefix,row,label,color in [('bounded',b,'Frozen baseline','C1'),('napkin',c,'Updated mechanics','C0')]:
  a=np.load(O/f"{prefix}-{day}-nominal-5-{row['start']}.npz");t=a['t'];m=t>=0;y=a['states'][m];truth=a['truth'][m];v=y[:,7:10];track=a['logged_position'][m];track-=track[0]
  axes[0,col].plot(y[:,1]-y[0,1],y[:,0]-y[0,0],label=label,color=color)
  axes[1,col].plot(t[m],np.rad2deg(R.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),color=color)
  axes[2,col].plot(t[m],-v[:,2],color=color)
 axes[0,col].plot(track[:,1],track[:,0],color='k',label='Flight');axes[1,col].plot(t[m],np.rad2deg(truth[:,0]),color='k');axes[2,col].plot(t[m],-truth[:,8],color='k')
 axes[0,col].set_title('Hood River' if col==0 else 'Trout Lake');axes[0,col].set(xlabel='East displacement (m)',ylabel='North displacement (m)');axes[0,col].axis('equal');axes[0,col].legend()
 axes[1,col].set(ylabel='Payload bank (deg)',xlabel='Forecast time (s)');axes[2,col].set(ylabel='Climb (m/s)',xlabel='Forecast time (s)')
for ax in axes.flat:ax.grid(alpha=.25)
fig.suptitle('Mechanics-only ablation: coefficients frozen, 5 s lead-in / 10 s forecast')
fig.savefig(O/'napkin-mechanics-comparison.png',dpi=150)
(O/'napkin-mechanics-score.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
