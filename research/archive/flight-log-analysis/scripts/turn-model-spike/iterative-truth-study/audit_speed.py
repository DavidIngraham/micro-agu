from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'frozen-atlas.json').read_text())['rows'];scores=json.loads((R/'cycle2-result.json').read_text())['scores'];data=np.load(R/'atlas-data.npz');pred=np.load(R/'cycle2-predictions.npz');out={};fig,ax=plt.subplots(2,1,figsize=(12,7))
for aa,day in zip(ax,['2026-01-03','2026-02-15']):
 s=[x for x in scores if x['day']==day and x['tier']=='ordinary'];rr=[]
 for x in s:
  k=x['index'];rr.append(dict(start=rows[k]['start_s'],index=k,stratum=x['stratum'],observed=float(data[f'{k}_observed'][:,0].mean()),predicted=float(pred[str(k)][:,0].mean()),bias=x['speed_bias']))
 out[day]=rr;aa.plot([x['start'] for x in rr],[x['observed'] for x in rr],'ko-',label='observed mean ground speed');aa.plot([x['start'] for x in rr],[x['predicted'] for x in rr],'o-',label='C2');aa.set_title(day);aa.set_ylabel('m/s');aa.grid(alpha=.2);aa.legend()
ax[-1].set_xlabel('Boot seconds');fig.tight_layout();fig.savefig(R/'speed-audit.png',dpi=140);(R/'speed-audit.json').write_text(json.dumps(out,indent=2));print(json.dumps({d:{s:float(np.sqrt(np.mean([x['bias']**2 for x in rr if x['stratum']==s]))) for s in sorted(set(x['stratum'] for x in rr))} for d,rr in out.items()},indent=2))
