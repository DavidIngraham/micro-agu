from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;d=np.load(R/'atlas-data.npz');z=np.load(R/'cycle2-predictions.npz');r=json.loads((R/'cycle2-result.json').read_text());rows=json.loads((R/'frozen-atlas.json').read_text())['rows'];fig,axs=plt.subplots(2,3,figsize=(15,8))
fail=[s['index'] for s in r['scores'] if s['tier']=='ordinary' and not s['guard_pass']];ordinary=[s for s in r['scores'] if s['tier']=='ordinary'];best=sorted(ordinary,key=lambda s:s['E']-s['baseline_E'])[:3]
for ax,k in zip(axs.flat,fail+[s['index'] for s in best]):
 t=d[f'{k}_time'];ax.plot(t,d[f'{k}_observed'][:,2],'k',label='flight');ax.plot(t,np.rad2deg(d[f'{k}_baseline'][:,2]),label='A1');ax.plot(t,np.rad2deg(z[str(k)][:,2]),label='cycle2');ax.set_title(f"{rows[k]['day']} {rows[k]['start_s']}s / "+('regression' if k in fail else 'improvement'));ax.set_xlabel('Seconds');ax.set_ylabel('Relative course deg');ax.grid(alpha=.2);ax.legend()
fig.tight_layout();fig.savefig(R/'cycle2-course-comparison.png',dpi=140)
c=json.loads((R/'cycle1-result.json').read_text());c['decision']='Retain provisionally after practical_model_review: calibration improvement, not identified zero physical coupling.';(R/'cycle1-result.json').write_text(json.dumps(c,indent=2));variant=dict(r['parameters']);variant['sink_brake']=c['baseline_parameters']['sink_brake'];variant['sink_turn']=c['baseline_parameters']['sink_turn'];(R/'nonzero-coupling-sensitivity.json').write_text(json.dumps(dict(parameters=variant,purpose='Sensitivity control preserving A1 brake and turn sink penalties; not a refitted or accepted candidate.'),indent=2))
