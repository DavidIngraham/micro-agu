from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;d=np.load(R/'atlas-data.npz');z=np.load(R/'cycle3-predictions.npz');r=json.loads((R/'cycle3-result.json').read_text());rows=json.loads((R/'frozen-atlas.json').read_text())['rows'];fig,axs=plt.subplots(2,3,figsize=(15,8))
fail=[s['index'] for s in r['scores'] if s['tier']=='ordinary' and not s['guard_pass']];ordinary=[s for s in r['scores'] if s['tier']=='ordinary'];best=sorted(ordinary,key=lambda s:s['E']-s['baseline_E'])[:3]
for ax,k in zip(axs.flat,fail+[s['index'] for s in best]):
 t=d[f'{k}_time'];ax.plot(t,d[f'{k}_observed'][:,2],'k',label='flight');ax.plot(t,np.rad2deg(d[f'{k}_baseline'][:,2]),label='A1');ax.plot(t,np.rad2deg(z[str(k)][:,2]),label='cycle3');ax.set_title(f"{rows[k]['day']} {rows[k]['start_s']}s / "+('regression' if k in fail else 'improvement'));ax.set_xlabel('Seconds');ax.set_ylabel('Relative course deg');ax.grid(alpha=.2);ax.legend()
fig.tight_layout();fig.savefig(R/'cycle3-course-comparison.png',dpi=140)

