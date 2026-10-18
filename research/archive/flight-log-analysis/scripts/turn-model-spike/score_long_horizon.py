from pathlib import Path
import json,sys,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
O=Path(__file__).parent;mode=sys.argv[1] if len(sys.argv)>1 else 'candidate';plan=json.loads((O/'net-improvement-plan.json').read_text());extra=json.loads((O/'long-horizon-refit-plan.json').read_text())['additional_moderate_checks'];sets={m:json.loads((O/f'broader-{m}-results.json').read_text()) for m in ['baseline',mode]};keys=plan['metrics'];floors=plan['floors'];rows=[];selected={}
for day,cut in plan['broader']+extra:
 chosen={}
 for mode,data in sets.items():
  group=[r for r in data if r['day']==day and r['cutoff_s']==cut];assert len(group)==3,(mode,day,cut,len(group));chosen[mode]=min(group,key=lambda r:r['cost'])
 selected[(day,cut)]=chosen
 b,c=chosen['baseline'],chosen[mode];ratios=[c['forecast'][k]/max(b['forecast'][k],floor) if c['status']=='finite' and b['status']=='finite' else float('inf') for k,floor in zip(keys,floors)];rows.append(dict(day=day,cutoff_s=cut,baseline=b['forecast'],candidate=c['forecast'],ratios=ratios,score=float(np.mean(ratios)),individual_metrics_pass=bool(np.all(np.array(ratios)<=1.1))))
def decision(subset):
 rr=[r for r in rows if [r['day'],r['cutoff_s']] in subset];per={d:float(np.mean([r['score'] for r in rr if r['day']==d])) for d in ['2026-01-03','2026-02-15']};aggregate=float(np.mean(list(per.values())));return dict(aggregate=aggregate,per_flight=per,pass_gate=bool(aggregate<=.8 and max(per.values())<=1 and all(r['individual_metrics_pass'] for r in rr)),max_metric_ratio=float(max(max(r['ratios']) for r in rr)))
report=dict(initial=decision(plan['initial']),broader=decision(plan['broader']+extra),rows=rows,selection='Minimum lead-in objective; divergent selected forecasts fail, never replaced with a favorable start.');(O/f'{mode}-refit-score.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
fig,ax=plt.subplots(4,4,figsize=(17,14),constrained_layout=True)
for col,(day,cut) in enumerate(plan['broader']+extra):
 row=col//4;col=col%4;trackax=ax[2*row,col];bankax=ax[2*row+1,col];chosen=selected[(day,cut)]
 for mode,color in [('baseline','C1'),(mode,'C0')]:
  r=chosen[mode]
  if r['status']!='finite':
   trackax.text(.03,.97,f'{mode}: diverged',transform=trackax.transAxes,va='top',color=color);continue
  a=np.load(O/f"broader-{mode}-{day}-{int(cut)}-nominal-5-{r['start']}.npz");t=a['t'];m=t>=0;y=a['states'][m];logged=a['logged_position'][m];logged=logged-logged[0];trackax.plot(y[:,1]-y[0,1],y[:,0]-y[0,0],color=color,label=mode);bankax.plot(t[m],np.rad2deg(R.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),color=color);truth=a['truth'][m]
 trackax.plot(logged[:,1],logged[:,0],color='k',label='Flight');bankax.plot(t[m],np.rad2deg(truth[:,0]),color='k');trackax.set_title(f"{'HR' if day.endswith('03') else 'TL'} {cut} s");trackax.set(xlabel='East (m)',ylabel='North (m)');trackax.axis('equal');bankax.set(xlabel='Forecast (s)',ylabel='Bank (deg)');trackax.legend(fontsize=8)
for ar in ax.flat:ar.grid(alpha=.25)
fig.suptitle('Retrospective development forecasts: candidate vs frozen baseline');fig.savefig(O/f'{mode}-refit-forecasts.png',dpi=130)

