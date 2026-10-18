from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'frozen-atlas.json').read_text())['rows'];out=[]
for cycle in [2,5]:
 z=np.load(R/f'cycle{cycle}-predictions.npz');cache={}
 for k,r in enumerate(rows):
  if r['tier']!='ordinary':continue
  day=r['day']
  if day not in cache:cache[day]=np.load(R.parent/f'{day}-causal-input.npz')
  a=cache[day];i=np.searchsorted(a['t'],r['start_s']);y=a['Y'][i:i+751];w=a['wind'][i,:2];av=y[:,6:8]-w;v=np.linalg.norm(av,axis=1);unit=av/np.maximum(v[:,None],.1);zz=z[str(k)];hybrid=np.linalg.norm(zz[:,3,None]*unit+w,axis=1);gs=np.linalg.norm(y[:,6:8],axis=1)
  out.append(dict(cycle=cycle,day=day,index=k,start=r['start_s'],air_bias=float((zz[:,3]-v).mean()),hybrid_ground_bias=float((hybrid-gs).mean()),coupled_ground_bias=float((zz[:,0]-gs).mean())))
summary={}
for cycle in [2,5]:
 for day in cache:
  rr=[s for s in out if s['cycle']==cycle and s['day']==day];summary[f'C{cycle} {day}']={key:float(np.sqrt(np.mean([s[key]**2 for s in rr]))) for key in ['air_bias','hybrid_ground_bias','coupled_ground_bias']}
(R/'speed-decomposition.json').write_text(json.dumps(dict(note='Hybrid diagnostic holds measured wind-inferred direction, changing only speed. Neither hybrid nor inferred airspeed is validation.',summary=summary,rows=out),indent=2));fig,ax=plt.subplots(figsize=(12,5));labels=list(summary);x=np.arange(len(labels))
for j,key in enumerate(['air_bias','hybrid_ground_bias','coupled_ground_bias']):ax.bar(x+(j-1)*.25,[summary[k][key] for k in labels],width=.25,label=key)
ax.axhline(.6,color='k',ls='--',label='ground-speed acceptance limit');ax.set_xticks(x,labels);ax.set_ylabel('RMS episode-mean error (m/s)');ax.legend();fig.tight_layout();fig.savefig(R/'speed-decomposition.png',dpi=140);print(json.dumps(summary,indent=2))
