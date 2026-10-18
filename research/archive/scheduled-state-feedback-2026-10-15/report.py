"""Render published figures and audit that all results use the final gain file."""
import hashlib
import json
import platform
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from experiment import HERE, save, DT

COLORS=['#286b89','#c36535','#348256']
NAMES=['baseline','candidate','scheduled']
LABELS=['Conventional baseline','Conventional candidate','Scheduled LQI']

def trace(case,name):return np.load(HERE/'traces'/f'{case}-{name}.npz')['trace']

def main():
    rows=json.loads((HERE/'results.json').read_text())
    observer=json.loads((HERE/'observer-results.json').read_text())
    ablation=json.loads((HERE/'ablation-results.json').read_text())
    gainhash=hashlib.sha256((HERE/'gains.npz').read_bytes()).hexdigest()
    assert len(rows)==24 and len(observer)==12 and len(ablation)==4
    for r in rows+observer+ablation:
        assert r['gains_sha256']==gainhash
        d=trace(r['case'],r['controller'])
        assert np.all(np.isfinite(d)) and np.all(d[:,9]>=0) and np.all(d[:,14]>=-1e-10)
        # Independently recompute headline error and pitch statistics from saved traces.
        w=d[:,0]>= (5 if r['case'] in ['maneuver','speed_transition','limits_recovery'] else 15)
        assert np.isclose(r['height_rms'],np.sqrt(np.mean((d[w,1]-d[w,2])**2)))
        assert np.isclose(r['pitch_rms'],np.sqrt(np.mean(d[w,7]**2)))
    fig,ax=plt.subplots(3,3,figsize=(14,10),layout='constrained')
    for col,case in enumerate(['maneuver','gust','voltage_ramp']):
        for name,label,color in zip(NAMES,LABELS,COLORS):
            d=trace(case,name)
            for row,values in enumerate([d[:,1]-d[:,2],d[:,7],100*d[:,9]]):
                ax[row,col].plot(d[:,0],values,label=label,color=color,lw=1)
        ax[0,col].set_title({'maneuver':'Climb and turn request','gust':'Matched gust history','voltage_ramp':'Gusts + 22.2 → 20.4 V'}[case])
        ax[2,col].plot(d[:,0],100*(d[:,9]+d[:,14]),'k--',lw=.8,label='Throttle ceiling')
        ax[2,col].set_xlabel('Time (s)')
        for row,label in enumerate(['Height error (m)','Payload pitch rate (deg/s)','Throttle (%)']):
            ax[row,col].set_ylabel(label);ax[row,col].grid(alpha=.2)
    ax[0,0].legend(fontsize=8)
    fig.suptitle('New comparison: common reference shaping, actuator limits and applied-command anti-windup')
    fig.savefig(HERE/'matched.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(3,2,figsize=(12,9),layout='constrained')
    for col,case in enumerate(['speed_transition','limits_recovery']):
        for name,label,color in zip(NAMES,LABELS,COLORS):
            d=trace(case,name)
            for row,key in enumerate([1,12,9]):ax[row,col].plot(d[:,0],d[:,key],label=label,color=color,lw=1)
        ax[0,col].plot(d[:,0],d[:,2],'k--',label='Shaped height request')
        ax[0,col].set_title('Climb, descent and turning' if col==0 else 'Limited request and recovery; gusts stop at 50 s')
        for row,label in enumerate(['Height (m)','Airspeed (m/s)','Throttle fraction']):ax[row,col].set_ylabel(label);ax[row,col].grid(alpha=.2)
        ax[2,col].set_xlabel('Time (s)')
    ax[0,0].legend(fontsize=8);fig.savefig(HERE/'transitions.png',dpi=160);plt.close(fig)
    cases=['maneuver','gust','soft_roll_gust','voltage_ramp']
    fig,ax=plt.subplots(1,3,figsize=(13,4.5),layout='constrained')
    for j,(name,label,color) in enumerate(zip(NAMES,LABELS,COLORS)):
        for a,key,title in zip(ax,['height_rms','pitch_rms','heading_rms'],['Height error RMS (m)','Pitch-rate RMS (deg/s)','Heading-rate error RMS (deg/s)']):
            values=[next(r[key] for r in observer if r['case']==c and r['controller']==name+'_observer') for c in cases]
            a.bar(np.arange(4)+(j-1)*.25,values,width=.25,color=color,label=label)
            a.set_xticks(range(4),['Maneuver','Gust','Soft roll','Voltage\nramp']);a.set_title(title);a.grid(axis='y',alpha=.2)
    ax[0].legend(fontsize=8);fig.suptitle('Shared model-based observer: noisy payload sensors; navigation at 5 Hz; no canopy sensors')
    fig.savefig(HERE/'observer.png',dpi=160);plt.close(fig)
    provenance=dict(python=platform.python_version(),gain_sha256=gainhash,
        source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.glob('*.py')},
        completed_runs=len(rows)+len(observer)+len(ablation),all_finite=True,
        note='No native scheduled-controller runs. Original native tests concern the old conventional tunes only.')
    save('provenance.json',provenance)
    print('Final result audit passed:',provenance['completed_runs'],'runs')
    for source,title in [(rows,'TRUTH'),(observer,'OBSERVER'),(ablation,'ABLATIONS')]:
        print(title)
        for r in source:print(r['case'],r['controller'],*[round(r[k],4) for k in ['height_rms','heading_rms','pitch_rms','throttle_std','ceiling_fraction','tail_height_rms']])

if __name__=='__main__':main()
