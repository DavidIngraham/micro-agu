"""Final, frozen direct-feedback validation with accelerometer-equipped estimation."""
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from experiment import HERE,save

def main():
    rows=json.loads((HERE/'iteration2/final-validation.json').read_text())
    selected=json.loads((HERE/'iteration2/refined-codesign-selection.json').read_text())
    assert len(rows)==48 and selected['selected']['passes']
    comparisons=[]
    for r in rows:
        d=np.load(HERE/'traces'/f'{r["case"]}-{r["controller"]}.npz')['trace']
        assert np.isfinite(d).all() and np.all(d[:,9]>=0) and np.all(d[:,14]>=-1e-9)
        use=d[:,0]>=(5 if r['case'] in ['maneuver','speed_transition','limits_recovery'] else 15)
        for k,value in [('height_rms',np.sqrt(np.mean((d[use,1]-d[use,2])**2))),('pitch_rms',np.sqrt(np.mean(d[use,7]**2))),('heading_rms',np.sqrt(np.mean((d[use,5]-d[use,6])**2)))]:assert np.isclose(r[k],value)
        if r['family']=='direct':
            b=next(b for b in rows if b['case']==r['case'] and b['observer']==r['observer'] and b['family']=='baseline')
            ratio={k:r[k]/b[k] for k in ['height_rms','pitch_rms','heading_rms']}
            comparisons.append(dict(case=r['case'],observer=r['observer'],ratios=ratio,
                passes=bool(ratio['height_rms']<=1.05 and ratio['pitch_rms']<=1.05 and ratio['heading_rms']<=1.1)))
    audit=dict(runs=48,all_finite=True,development_passes=True,all_validation_pass=sum(r['passes'] for r in comparisons)==16,
               passed_comparisons=sum(r['passes'] for r in comparisons),comparisons=16,details=comparisons,
               source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')})
    save('iteration2/final-audit.json',audit)
    colors=['#286b89','#c36535','#348256'];families=['baseline','candidate','direct']
    labels=['Conventional baseline','Conventional candidate','Refined direct state feedback']
    cases=['maneuver','gust','soft_roll_gust','voltage_ramp']
    fig,ax=plt.subplots(2,3,figsize=(13,8),layout='constrained')
    for line,obs in enumerate([False,True]):
        for j,(family,label,color) in enumerate(zip(families,labels,colors)):
            for a,key,title in zip(ax[line],['height_rms','pitch_rms','heading_rms'],['Height error RMS (m)','Pitch-rate RMS (deg/s)','Heading-rate error RMS (deg/s)']):
                values=[next(r[key] for r in rows if r['case']==c and r['observer']==obs and r['family']==family) for c in cases]
                a.bar(np.arange(4)+(j-1)*.25,values,width=.25,label=label,color=color)
                a.set_xticks(range(4),['New\nmaneuver','New\ngust','Softer-roll\nmismatch','Voltage\nramp']);a.set_title(title);a.grid(axis='y',alpha=.2)
        ax[line,0].set_ylabel('IMU + navigation observer' if obs else 'Exact states')
    handles,labels=ax[0,0].get_legend_handles_labels();fig.legend(handles,labels,loc='outside upper center',ncol=3,fontsize=10)
    fig.savefig(HERE/'iteration2/final-validation.png',dpi=160);plt.close(fig)
    print('Acceptance checks:',audit['passed_comparisons'],'/16')
    for obs in [False,True]:
        print('OBSERVER' if obs else 'TRUTH')
        for case in [c['case'] for c in comparisons if c['observer']==obs]:
            for f in ['baseline','direct']:
                r=next(r for r in rows if r['case']==case and r['observer']==obs and r['family']==f)
                print(case,f,*[round(r[k],4) for k in ['height_rms','pitch_rms','heading_rms','ceiling_fraction','tail_height_rms']])

if __name__=='__main__':main()
