from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;rows=json.loads((R/'roll-mode-sensitivity.json').read_text());obs=json.loads((R/'observed-mode-shapes.json').read_text());fig,ax=plt.subplots(2,2,figsize=(12,8))
for di,(flight,day,start) in enumerate([('hood_river','2026-01-03',480),('trout_lake','2026-02-15',2790)]):
 for j,branch in enumerate(['low','high']):
  o=next(r for r in obs if r['day']==day and r['start']==start and r['branch']==branch);x=np.arange(3);ax[di,j].bar(x-.25,o['amplitude'],width=.25,label=f'flight ridge {o["hz"]:.2f}Hz',color='.3')
  for shift,kr,color in [(0,4.8622,'C0'),(.25,1.,'C1')]:
   r=next(r for r in rows if r['flight']==flight and r['kr']==kr);m=next(m for m in r['matches'] if m['branch']==branch);v=np.array(m['payload_rate_mode_abs']);v/=np.linalg.norm(v);ax[di,j].bar(x+shift,v,width=.25,label=f'kroll {kr:.2f}, {m["frequency_hz"]:.2f}Hz',color=color)
  ax[di,j].set_xticks(x,['Roll','Pitch','Yaw']);ax[di,j].set(title=f'{flight}: {branch} branch',ylabel='Normalized rate amplitude',ylim=(0,1));ax[di,j].legend(fontsize=8);ax[di,j].grid(axis='y',alpha=.2)
fig.suptitle('Lower roll stiffness mixes roll into the high-frequency coupled mode');fig.tight_layout();fig.savefig(R/'roll-mode-shape-comparison.png',dpi=150)
