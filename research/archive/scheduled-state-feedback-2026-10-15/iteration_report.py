"""Audit and plot the frozen second-pass controller on new disturbance realizations."""
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from experiment import HERE,save

def main():
    rows=json.loads((HERE/'iteration2/validation-results.json').read_text())
    selection=json.loads((HERE/'iteration2/hybrid-selection.json').read_text())
    assert len(rows)==48
    ratios=[]
    for row in rows:
        d=np.load(HERE/'traces'/f'{row["case"]}-{row["controller"]}.npz')['trace']
        assert np.isfinite(d).all() and np.all(d[:,9]>=0) and np.all(d[:,14]>=-1e-9)
        window=d[:,0]>=(5 if row['case'] in ['maneuver','speed_transition','limits_recovery'] else 15)
        assert np.isclose(row['height_rms'],np.sqrt(np.mean((d[window,1]-d[window,2])**2)))
        if row['family']=='scheduled':
            base=next(r for r in rows if r['case']==row['case'] and r['observer']==row['observer'] and r['family']=='baseline')
            first=next(r for r in rows if r['case']==row['case'] and r['observer']==row['observer'] and r['family']=='initial')
            ratios.append(dict(case=row['case'],observer=row['observer'],
                baseline_ratios={k:row[k]/base[k] for k in ['height_rms','pitch_rms','heading_rms']},
                initial_ratios={k:row[k]/first[k] for k in ['height_rms','pitch_rms','heading_rms']}))
    passes=all(r['baseline_ratios']['height_rms']<=1.05 and r['baseline_ratios']['pitch_rms']<=1.05 and r['baseline_ratios']['heading_rms']<=1.1 for r in ratios)
    save('iteration2/validation-audit.json',dict(runs=len(rows),finite_and_throttle_bounds=True,
        development_all_constraints_met=selection['all_constraints_met'],validation_all_constraints_met=passes,comparisons=ratios,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')}))
    fig,ax=plt.subplots(2,3,figsize=(13,8),layout='constrained')
    cases=['maneuver','gust','soft_roll_gust','voltage_ramp']
    for line,obs in enumerate([False,True]):
        for j,(name,label,color) in enumerate(zip(['baseline','initial','scheduled'],['Conventional baseline','Direct scheduled LQI','Bounded state correction'],['#286b89','#c36535','#348256'])):
            for a,key,title in zip(ax[line],['height_rms','pitch_rms','heading_rms'],['Height error RMS (m)','Pitch-rate RMS (deg/s)','Heading-rate error RMS (deg/s)']):
                values=[next(r[key] for r in rows if r['case']==c and r['observer']==obs and r['family']==name) for c in cases]
                a.bar(np.arange(4)+(j-1)*.25,values,width=.25,label=label,color=color)
                a.set_xticks(range(4),['Maneuver','Gust','Soft roll','Voltage\nramp']);a.set_title(title);a.grid(axis='y',alpha=.2)
        ax[line,0].set_ylabel('Estimated states' if obs else 'Exact states')
    handles,labels=ax[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside upper center',ncol=3,fontsize=10)
    fig.savefig(HERE/'iteration2/validation.png',dpi=160);plt.close(fig)
    for obs in [False,True]:
        print('OBSERVER' if obs else 'TRUTH')
        for case in cases:
            for name in ['baseline','initial','scheduled']:
                r=next(r for r in rows if r['case']==case and r['observer']==obs and r['family']==name)
                print(case,name,*[round(r[k],4) for k in ['height_rms','pitch_rms','heading_rms']])
    print('Development criteria all met:',selection['all_constraints_met'],'Validation criteria all met:',passes)

if __name__=='__main__':main()
