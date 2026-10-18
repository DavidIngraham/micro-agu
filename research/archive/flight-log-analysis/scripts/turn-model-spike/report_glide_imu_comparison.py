from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
O=Path(__file__).parent;plan=json.loads((O/'glide-comparison-plan.json').read_text());scoreplan=json.loads((O/'net-improvement-plan.json').read_text());sets={m:json.loads((O/f'glide-imu-{m}-results.json').read_text()) for m in ['baseline','mechanics']};rows=[];fig,axs=plt.subplots(3,2,figsize=(12,9))
for col,man in enumerate(plan['maneuvers']):
 day=man['day'];cut=man['cutoff_s'];selected={}
 for mode in sets:
  group=[r for r in sets[mode] if r['day']==day];assert len(group)==3,(mode,day,len(group));r=min(group,key=lambda x:x['cost']);selected[mode]=r
  if r['status']!='finite':continue
  z=np.load(O/f"glide-imu-{mode}-{day}-{int(cut)}-nominal-5-{r['start']}.npz");sl=z['t']>=0;t=z['t'][sl];y=z['states'][sl];truth=z['truth'][sl];axs[0,col].plot(t,-y[:,9],label=mode);axs[1,col].plot(t,np.linalg.norm(y[:,7:9],axis=1),label=mode);axs[2,col].plot(t,np.rad2deg(Rotation.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),label=mode)
 axs[0,col].plot(t,-truth[:,8],'k',label='Flight');axs[1,col].plot(t,np.linalg.norm(truth[:,6:8],axis=1),'k',label='Flight');axs[2,col].plot(t,np.rad2deg(truth[:,0]),'k',label='Flight')
 b,c=selected['baseline'],selected['mechanics'];ratios=[c['forecast'][k]/max(b['forecast'][k],f) for k,f in zip(scoreplan['metrics'],scoreplan['floors'])];rows.append(dict(day=day,baseline=b['forecast'],mechanics=c['forecast'],ratios=ratios,score=float(np.mean(ratios)),lead_cost_baseline=b['cost'],lead_cost_mechanics=c['cost']))
 axs[0,col].set_title(day)
 for row in range(3):axs[row,col].grid(alpha=.2);axs[row,col].legend(fontsize=8);axs[row,col].set_xlabel('Forecast time (s)')
 for ax,label in zip(axs[:,col],['Climb (m/s)','Groundspeed (m/s)','Payload bank (deg)']):ax.set_ylabel(label)
fig.suptitle('Low-power diagnostic with IMU-informed initialization');fig.tight_layout();fig.savefig(O/'glide-imu-comparison.png',dpi=160)
out=dict(scope='Retrospective low-power diagnostic, not a substitute for original acceptance set. Same normalized metrics/floors; state selection uses lead-in only.',metrics=scoreplan['metrics'],aggregate=float(np.mean([r['score'] for r in rows])),rows=rows);(O/'glide-imu-comparison-score.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))

