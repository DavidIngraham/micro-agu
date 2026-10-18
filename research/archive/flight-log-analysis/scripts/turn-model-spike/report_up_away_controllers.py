from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;r=json.loads((O/'up-away-controller-results.json').read_text());tr=np.load(O/'up-away-controller-nominal.npz')['trace'];fig,ax=plt.subplots(3,2,figsize=(13,9))
for d in [0,1]:
 z=tr[:,d];t=z[:,0];ax[0,d].plot(t,z[:,3],label='air turn rate');ax[0,d].plot(t,z[:,4],'--',label='command');ax[0,d].set_ylabel('deg/s');ax[0,d].set_title(['1.385kg','1.758kg'][d]);ax[1,d].plot(t,z[:,2],label='climb');target=z[:,5].copy();target[(t>=300)&(t<340)]=np.nan;ax[1,d].plot(t,target,'--',label='climb command');ax[1,d].set_ylabel('m/s');ax[2,d].plot(t,z[:,6],label='throttle');ax[2,d].plot(t,z[:,7],label='differential brake');ax[2,d].set_ylabel('fraction');ax[2,d].set_xlabel('seconds')
 for a in ax[:,d]:a.axvspan(240,260,color='orange',alpha=.15);a.axvspan(300,340,color='gray',alpha=.15);a.grid(alpha=.2);a.legend(fontsize=8)
fig.suptitle('Diagnostic controller cores on unaccepted A1 plant | orange: saturation request; gray: glide');fig.tight_layout();fig.savefig(O/'up-away-controller-diagnostic.png',dpi=160)
summary=dict(cases=len(r)*2,all_finite=all(x['finite'] for x in r),max_turn_integral=max(x['max_turn_integral'] for x in r),max_climb_integral=max(x['max_climb_integral'] for x in r),nominal_segments=next(x['segments'] for x in r if x['name']=='A1'),interpretation='Finite controller core runs do not validate plant fidelity or full controller. Wind variants exercise ground-track drift only; no wind-compensating path-guidance loop is present. Glide climb error in raw results is relative to placeholder0, not tracking a glide-climb target.')
(O/'up-away-controller-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
