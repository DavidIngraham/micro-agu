from pathlib import Path
import json,hashlib,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent; rows=[]; excluded={}; arrays={}
for day in ['2026-01-03','2026-02-15']:
 a=np.load(O/f'{day}-causal-input.npz'); b=np.load(O/f'{day}.npz');t=a['t'];y=a['Y'];u=a['U']; air=np.interp(t,b['t'],b['airborne'].astype(float))>.99
 for start in range(25,int(t[-1])-15,20):
  i=np.searchsorted(t,start); sl=slice(i,i+251); lead=slice(i-125,i+251)
  reason=None
  if i<125 or i+251>len(t):reason='edge'
  elif not np.all(a['valid'][lead]) or not np.all(air[lead]):reason='invalid or not airborne including lead'
  elif np.max(np.abs(np.rad2deg(y[lead,0])))>35:reason='bank above35'
  elif np.min(np.linalg.norm(y[sl,6:8],axis=1))<2:reason='ground speed below2'
  # A 5s lead plus 10s forecast must remain inside a single 60s partition.
  elif int((t[i]-5)//60)!=int(t[i+250]//60):reason='partition guard'
  if reason:excluded[reason]=excluded.get(reason,0)+1;continue
  z=y[sl]; uu=u[sl]; speed=np.linalg.norm(z[:,6:8],axis=1);chi=np.unwrap(np.arctan2(z[:,7],z[:,6]));q=np.rad2deg(chi[-1]-chi[0])/10; c=-z[:,8]
  cat='glide' if np.quantile(uu[:,0],.9)<.1 else ('left' if q < -3 else 'right' if q>3 else 'straight')
  transition=bool(np.ptp(uu[:,0])>.08 or max(np.ptp(uu[:,1:],axis=0))*72>12)
  r=dict(day=day,start_s=float(t[i]),partition='check' if int(t[i]//60)%3==2 else 'development',category=cat,transition=transition,speed=float(speed.mean()),climb=float(c.mean()),turn=float(q),throttle=float(uu[:,0].mean()),left_mm=float(uu[:,1].mean()*72),right_mm=float(uu[:,2].mean()*72),half_change=[float(speed[125:].mean()-speed[:125].mean()),float(c[125:].mean()-c[:125].mean()),float(np.rad2deg((chi[-1]-chi[125])-(chi[125]-chi[0]))/5)])
  rows.append(r)
summary={}
for d in ['2026-01-03','2026-02-15']:
 summary[d]={p:{c:sum(r['day']==d and r['partition']==p and r['category']==c for r in rows) for c in ['glide','straight','left','right']} for p in ['development','check']}
# These are within-window variability diagnostics, not measurement noise or independent repeatability.
variability={d:dict(all_windows_median_abs_half_change=np.median(np.abs([r['half_change'] for r in rows if r['day']==d]),axis=0).tolist(),low_input_variation_count=sum(r['day']==d and not r['transition'] for r in rows)) for d in summary}
manifest=dict(status='Coverage candidate; freeze only after coverage deficiencies addressed, before fitting',rules='10s blocks every20s starting25s; valid airborne 5s lead; peak bank<=35deg; min ground speed2m/s; minute mod3==2 check, others development; entire lead+forecast inside minute',measurement='EKF ground velocity; course is GNSS-derived, not body yaw; no airspeed claims',summary=summary,within_window_variability_not_noise=variability,excluded=excluded,rows=rows,inputs={f:hashlib.sha256((O/f).read_bytes()).hexdigest() for d in summary for f in [f'{d}-causal-input.npz',f'{d}.npz']})
(O/'up-away-episode-candidate.json').write_text(json.dumps(manifest,indent=2))
fig,ax=plt.subplots(2,3,figsize=(13,7))
for j,d in enumerate(summary):
 rr=[r for r in rows if r['day']==d]
 for p,marker in [('development','o'),('check','x')]:
  ss=[r for r in rr if r['partition']==p]
  for k,(xx,yy) in enumerate([('throttle','climb'),('right_mm','turn'),('start_s','turn')]):
   ax[j,k].scatter([r[xx] for r in ss],[r[yy] for r in ss],s=18,marker=marker,label=p,alpha=.7);ax[j,k].set_xlabel(xx);ax[j,k].set_ylabel(yy);ax[j,k].grid(alpha=.2)
 ax[j,0].set_title(d+' ground-flight coverage');ax[j,0].legend()
fig.suptitle('Candidate episodes selected without model residuals | 10-second means')
fig.tight_layout();fig.savefig(O/'up-away-episode-coverage.png',dpi=160)
print(json.dumps(dict(summary=summary,variability=variability,total=len(rows)),indent=2))

