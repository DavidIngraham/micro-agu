from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;s=json.loads((R/'baseline-scores.json').read_text());fig,axes=plt.subplots(2,1,figsize=(12,7),sharey=True)
for ax,day in zip(axes,['2026-01-03','2026-02-15']):
 for tier,color in [('ordinary','tab:blue'),('stress','tab:orange'),('glide-calibration','tab:green')]:
  rr=[x for x in s if x['day']==day and x['tier']==tier];ax.scatter([x['start_s'] for x in rr],[x['E'] for x in rr],label=tier,color=color)
 ax.set_title(day);ax.set_ylabel('Course error E (deg/s equivalent)');ax.grid(alpha=.2);ax.legend()
axes[-1].set_xlabel('Flight boot time (s)');fig.suptitle('Frozen continuous replay atlas — A1 baseline, before new tuning');fig.tight_layout();fig.savefig(R/'baseline-atlas-scores.png',dpi=150)
