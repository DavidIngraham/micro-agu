from pathlib import Path
import json,csv,datetime
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09');O=R/'system-identification'
fig,axs=plt.subplots(1,2,figsize=(12,5),layout='constrained')
for ax,day,label in zip(axs,['2026-01-03','2026-02-15'],['Hood River','Early Trout Lake']):
 f=json.loads((O/(day+'-steady-fit.json')).read_text());w=np.array(f['windows']);x=np.linspace(w[:,1].min(),w[:,1].max(),100)
 ax.scatter(w[:,1],w[:,2],label='10 s steady-throttle averages');ax.plot(x,f['slope']*x+f['intercept'],label='Local linear fit');ax.axhline(0,color='gray',lw=.8);ax.axvline(f['level_throttle'],ls='--',color='#20744c');ax.set_title(label+f" — level flight ≈ {f['level_throttle']:.1f}%");ax.set_xlabel('Throttle output (%)');ax.set_ylabel('Climb rate (m/s)');ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.savefig(O/'throttle-climb-fit.png',dpi=160)
ledger=[]
for d in sorted(R.glob('2026-*')):
 if not d.is_dir():continue
 hist={}
 for x in json.loads((d/'PARM.json').read_text()):
  k=x['Name'];v=round(x['Value'],5)
  if k.startswith(('TECS','THR_','TRIM_THROTTLE')):
   if k not in hist or hist[k][-1][1]!=v:hist.setdefault(k,[]).append((x['TimeUS']/1e6,v))
 for k,vs in hist.items():
  for i,(t,v) in enumerate(vs):ledger.append({'log':d.name,'parameter':k,'boot_seconds':round(t,3),'minutes_after_trout_takeoff':round((t-202.81)/60,3) if d.name=='2026-02-15 15-38-50' else '', 'previous':vs[i-1][1] if i else '', 'value':v,'event':'change' if i else 'initial'})
with (O/'longitudinal-parameter-ledger.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(ledger[0]));w.writeheader();w.writerows(ledger)
for row in json.loads((O/'limit-history.json').read_text()):
 if len(row['values'])>1 and row['source'].startswith('FIXED_WING'):
  print(row['source'],row['parameter'],[(datetime.datetime.fromtimestamp(float(t),datetime.timezone(datetime.timedelta(hours=-8))).strftime('%Y-%m-%d %H:%M:%S PST'),v) for t,v in row['values']])
# exact idle holds at Hood River
z=dict(np.load(R/'2026-01-03 15-41-42/aligned.npz'));m=(z['thr']<1)&z['airborne'];edges=np.flatnonzero(np.diff(np.r_[False,m,False]))
for lo,hi in zip(edges[::2],edges[1::2]):
 if hi-lo>75:print('Hood idle',z['t'][lo],z['t'][hi-1],np.mean(z['climb'][lo:hi]))
