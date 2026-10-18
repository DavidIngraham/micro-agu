from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;j=json.loads((O/'training-bank-coverage.json').read_text());f,axs=plt.subplots(1,2,figsize=(11,4),sharey=True)
for ax,day,label in zip(axs,['2026-01-03','2026-02-15'],['Hood River','Trout Lake']):
 x=np.arange(3)
 for off,row,color in [(-.18,j[0],'tab:blue'),(.18,j[1],'tab:orange')]:
  counts=[row['counts'].get(day+' '+tier,0) for tier in ['0–15','15–30','30+']];bars=ax.bar(x+off,counts,width=.36,label=row['set'],color=color);ax.bar_label(bars,padding=2)
 ax.set_xticks(x,['<15°','15–30°','≥30°']);ax.set(title=label,xlabel='Maximum bank in each 3 s segment',ylabel='Number of segments');ax.grid(axis='y',alpha=.2)
axs[0].legend();f.suptitle('The narrow physics-model training set contains no high-bank segments');f.tight_layout();f.savefig(O/'iteration-training-coverage.png',dpi=160)
