import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
O=Path(__file__).parent;rows=json.loads((O/'sustained-envelope-summary.json').read_text());fig,axes=plt.subplots(2,3,figsize=(14,8))
for j,day in enumerate(['2026-01-03','2026-02-15']):
 for model,style in [('baseline','-'),('mechanics','--')]:
  for throttle,color in [(0,'#426b9f'),(.4,'#d7832a'),(.6,'#448555')]:
   rr=sorted([r for r in rows if r['day']==day and r['model']==model and r['throttle']==throttle],key=lambda r:r['brakes_mm'][1]-r['brakes_mm'][0]);x=[r['brakes_mm'][1]-r['brakes_mm'][0] for r in rr]
   ys=[[r['radius_m'] if r['unique'] else np.nan for r in rr],[r['mean'][1] if r['unique'] else np.nan for r in rr],[r['mean'][2] if r['unique'] else np.nan for r in rr]]
   for k in range(3):axes[j,k].plot(x,ys[k],style,marker='o',color=color,label=f'{model} {throttle:.0%}')
 for k,title in enumerate(['Settled radius (m)','Settled climb (m/s)','Settled turn rate (deg/s)']):
  axes[j,k].set(title=f'{1.385 if j==0 else 1.758:.3f} kg — {title}',xlabel='Right minus left pull (mm)');axes[j,k].grid(alpha=.2)
axes[0,0].legend(fontsize=8);fig.suptitle('22.2V fixed controls: only cases agreeing across all three starts; gaps retained');fig.tight_layout();fig.savefig(O/'sustained-envelope-map.png',dpi=150)
matched=json.loads((O/'sustained-matched-comparison.json').read_text())['rows'];obs=[r['observed'] for r in matched[::2]];fig,axes=plt.subplots(1,3,figsize=(13,4.8));labels=[r['day'][5:]+' / '+str(r['start_s']) for r in obs]
for k,title in enumerate(['Airspeed (m/s, observed inferred)','Climb rate (m/s)','Air-relative turn (deg/s, observed inferred)']):
 values=[[r['inferred_mean_airspeed'],r['mean_ground_speed_climb_turn_bank'][1],r['inferred_net_air_turn_deg_s']][k] for r in obs];axes[k].plot(range(4),values,'ko',label='Flight average')
 for model,marker in [('baseline','s'),('mechanics','^')]:
  vals=[]
  for r in [r for r in matched if r['model']==model]:
   f=[x['mean_speed_climb_turn_bank'][k] for x in r['outcomes'] if x['status']=='finite'];vals.append(float(np.mean(f)) if f else np.nan)
  axes[k].plot(range(4),vals,marker,linestyle='none',label=model+' final average')
 axes[k].set_xticks(range(4),labels,rotation=25);axes[k].set_title(title);axes[k].grid(alpha=.2)
axes[0].legend();fig.suptitle('Actual mean controls/voltage: varying-control flight averages are not fixed-input equilibria');fig.tight_layout();fig.savefig(O/'sustained-matched-comparison.png',dpi=150)
print(json.dumps({model:dict(groups=sum(r['model']==model for r in rows),unique=sum(r['model']==model and r['unique'] for r in rows)) for model in ['baseline','mechanics']}))
