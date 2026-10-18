"""Plot native mission summaries without presenting unmatched gust phases as an ablation."""
import json
import numpy as np
from study import HERE, save
from analyze import COLORS
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    rows=json.loads((HERE/'native-results.json').read_text());assert len(rows)==4
    assert all(r['returncode']==0 and 'metrics' in r for r in rows)
    fig,ax=plt.subplots(1,3,figsize=(13,5),layout='constrained');summary=[]
    for j,name in enumerate(['baseline','candidate']):
        gust=next(r['metrics'] for r in rows if r['controller']==name and r['condition']=='gust')
        for a,key,title in zip(ax,['height_rms_m','pitch_rate_rms_deg_s','throttle_std_pct'],
                               ['Height error RMS (m)','Pitch-rate RMS (deg/s)','Throttle std (percentage points)']):
            a.bar(np.arange(2)+(j-.5)*.35,[gust['loiters'][k][key] for k in ['3','8']],width=.35,color=COLORS[j],label=name)
            a.set_xticks([0,1],['CW','CCW']);a.set_title(title);a.grid(axis='y',alpha=.2)
        for condition in ['calm','gust']:
            m=next(r['metrics'] for r in rows if r['controller']==name and r['condition']==condition)
            summary.append(dict(controller=name,condition=condition,max_waypoint_height_error_m=m['max_waypoint_height_error_m'],
                                max_corner_overshoot_m=m['max_corner_overshoot_m'],loiters=m['loiters']))
    ax[0].legend();fig.suptitle('Native gust missions: same generator, different route timing and gust phases')
    fig.savefig(HERE/'native.png',dpi=160);plt.close(fig);save('native-table.json',summary)
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
