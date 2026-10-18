import json,numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;d=json.loads((O/'up-away-long-duration.json').read_text());cases=d['cases'];fig,ax=plt.subplots(2,2,figsize=(10,7));summary=[]
for di,mass in enumerate([1.385,1.758]):
 for name in ['A1','B1']:
  r=next(r for r in d['results'] if r['model']==name and r['dt']==.04);v=np.array(r['final_means']);ix=[i for i,c in enumerate(cases) if c[0]==mass and c[2]==[0,0] and c[3]==4.5];u=[cases[i][1] for i in ix]
  ax[di,0].plot(u,v[ix,1],'-o',label=name);ax[di,1].plot(u,np.rad2deg(v[ix,2]),'-o',label=name)
 ax[di,0].set_title(str(mass)+'kg, zero commanded brake');ax[di,0].set_ylabel('Settled climb m/s');ax[di,1].set_ylabel('Settled air turn rate deg/s')
 for a in ax[di]:a.set_xlabel('Throttle fraction');a.grid(alpha=.2);a.legend()
fig.suptitle('Unaccepted models: finite trim is not necessarily correct trim');fig.tight_layout();fig.savefig(O/'up-away-numerical-trim.png',dpi=160)
summary=dict(total_runs=sum(r['cases'] for r in d['results']),all_finite=all(r['finite'] for r in d['results']),speed_clamp_total=sum(r['clamp_count'] for r in d['results']),max_settled_timestep_delta=np.max([r['timestep_mean_difference'] for r in d['results'] if 'timestep_mean_difference' in r],axis=0).tolist())
(O/'up-away-numerical-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
