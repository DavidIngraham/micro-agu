from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'modal-remaining-check.json').read_text());fig,ax=plt.subplots(2,2,figsize=(12,9));summary=[]
for di,day in enumerate(['2026-01-03','2026-02-15']):
 group=[r for r in rows if r['day']==day]
 for j,name in enumerate(['low','high']):
  good=[r for r in group if r[name]];ax[di,j].scatter([r['rpm'] for r in group],[r['observed_'+name] for r in group],label='observed spectral ridge',color='k',alpha=.6);sort=sorted(good,key=lambda r:r['rpm']);ax[di,j].plot([r['rpm'] for r in sort],[r[name]['frequency_hz'] for r in sort],label='local mode prediction',color='C1');dev=[r for r in good if r['subset']=='adjustment'];ax[di,j].scatter([r['rpm'] for r in dev],[r['observed_'+name] for r in dev],s=100,facecolors='none',edgecolors='C3',label='six adjustment windows');ax[di,j].set(title=f'{day} {name} branch',xlabel='Logged mean RPM',ylabel='Frequency (Hz)');ax[di,j].grid(alpha=.25)
 for subset in ['adjustment','remaining']:
  g=[r for r in group if r['subset']==subset];err=[r[k]['frequency_hz']-r['observed_'+k] for r in g for k in ['low','high'] if r[k]];summary.append(dict(day=day,subset=subset,windows=len(g),branches=len(err),rmse_hz=float(np.sqrt(np.mean(np.square(err)))),mean_error_hz=float(np.mean(err))))
ax[0,0].legend(fontsize=8);fig.suptitle('Shared rotor inertia and yaw attachment span; observed ridges are not identified poles');fig.tight_layout();fig.savefig(R/'modal-frequency-comparison.png',dpi=150);(R/'modal-frequency-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
