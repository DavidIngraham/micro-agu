from pathlib import Path
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
r=json.loads((O/'mission-performance-blocks.json').read_text())['blocks']
fig,ax=plt.subplots(1,2,figsize=(11,4),sharey=True)
cats=['glide','climb','turn_left','turn_right']
for a,day in zip(ax,['2026-01-03','2026-02-15']):
 for j,part in enumerate(['development','evaluation']):
  counts=[sum(x['day']==day and x['partition']==part and x['category']==c for x in r) for c in cats]
  a.bar([i+(j-.5)*.36 for i in range(4)],counts,width=.36,label=part)
 a.set_xticks(range(4),['Glide','Climb','Left turn','Right turn']);a.set_title(day);a.legend();a.grid(axis='y',alpha=.2)
ax[0].set_ylabel('Eligible nonoverlapping 10-second blocks')
fig.suptitle('Mission-model coverage audit — before fitting');fig.tight_layout();fig.savefig(O/'mission-performance-coverage.png',dpi=150)
