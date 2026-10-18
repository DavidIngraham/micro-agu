"""Show every held-out pair; no selection of representative scenarios."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/'validation/metrics.json').read_text())
fig,axes=plt.subplots(2,3,figsize=(12,7),layout='constrained')
metrics=[('Performance score (lower is better)',lambda r:r['score']),
         ('Climb command peak (°/s)',lambda r:r['command_peak_rates_deg_s'][0]),
         ('Modeled shaft energy (Wh/km)',lambda r:r['shaft_Wh_per_ground_km'])]
for row,cohort in enumerate(['local','deployed']):
    off={r['case']:r for r in rows if r['variant']==cohort+'_off' and r['score'] is not None}
    on={r['case']:r for r in rows if r['variant']==cohort+'_on' and r['score'] is not None}
    ids=sorted(off.keys() & on.keys())
    for ax,(label,fn) in zip(axes[row],metrics):
        for i in ids:
            ax.plot([0,1],[fn(off[i]),fn(on[i])],color='#879ab0',alpha=.65,marker='o',ms=3,lw=1)
        if ids:ax.plot([0,1],[np.median([fn(off[i]) for i in ids]),np.median([fn(on[i]) for i in ids])],color='#ce4f27',lw=3,marker='o',label='Median')
        ax.set(xticks=[0,1],xticklabels=['Filter off','0.7 s filter'],xlim=(-.2,1.2),title=label)
        ax.grid(axis='y',alpha=.2)
        if ax is axes[row,0]:ax.set_ylabel(f'{cohort.capitalize()} controller\n{len(ids)} matched validation cases')
        ax.spines[['top','right']].set_visible(False)
axes[0,0].legend(frameon=False)
fig.suptitle('Original-observer LQI: reference-filter backport\nSame gains and sensing within each cohort',fontsize=15)
fig.savefig(HERE/'validation-pairs.png',dpi=160)
fig.savefig(HERE/'validation-pairs.svg')
