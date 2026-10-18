"""Audit allocation-residual coupling to the turn and speed integrators."""
import ast
import json
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build import HERE
text=(HERE/'runtime/AP_CustomControl_LQI_gains.h').read_text()
m=re.search(r'float AW\[3\]\[3\] = ([^;]+)',text)
aw=np.array(ast.literal_eval(m[1].replace('{','[').replace('}',']').replace('f','')))
records=[]
paths=[p for cohort in ['screen','nominal','diagnostic'] for p in (HERE/cohort).glob('lqi-*/00[01].json')]
for p in sorted(paths):
    d=json.loads(p.read_text())
    if not d['complete']:continue
    for phase,start in [('turn1',40),('reverse',90),('turn2',180)]:
        a=np.array([s['diagnostic'] for s in d['samples'] if s['phase']==phase and s['time']>start and len(s['diagnostic'])==15])
        residual=a[:,5:8]-a[:,8:11];correction=residual@aw.T
        records.append(dict(rate=d['spec']['rate'],case=d['spec']['id'],phase=phase,reference_deg_s=float(np.mean(a[:,1])),measured_deg_s=float(np.mean(a[:,2])),turn_error_normalized=float(np.mean((a[:,2]-a[:,1])/10)),turn_antiwindup_correction=float(np.mean(correction[:,1])),speed_antiwindup_correction=float(np.mean(correction[:,2])),common_requested=float(np.mean(a[:,6])),common_applied=float(np.mean(a[:,9])),speed_integral=float(np.mean(a[:,13])),turn_integral=float(np.mean(a[:,12]))))
(HERE/'allocation-diagnosis.json').write_text(json.dumps(records,indent=2))
p=HERE/'nominal/lqi-15/000.json'
if p.exists():
    d=json.loads(p.read_text());ss=[s for s in d['samples'] if len(s['diagnostic'])==15];t=np.array([s['time'] for s in ss]);a=np.array([s['diagnostic'] for s in ss]);correction=(a[:,5:8]-a[:,8:11])@aw.T
    fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,layout='constrained')
    axes[0].plot(t,a[:,1],label='Reference');axes[0].plot(t,a[:,2],label='Measured');axes[0].set_ylabel('Heading rate (°/s)')
    axes[1].plot(t,a[:,6]*100,label='Requested');axes[1].plot(t,a[:,9]*100,label='Applied');axes[1].set_ylabel('Common brake (%)')
    axes[2].plot(t,(a[:,2]-a[:,1])/10,label='Normalized turn error');axes[2].plot(t,correction[:,1],label='Anti-windup correction');axes[2].set_ylabel('Turn integral input (1/s)')
    axes[3].plot(t,a[:,13],label='Speed integral');axes[3].axhline(50,color='r',linestyle='--',label='Integral bound');axes[3].set_ylabel('Normalized integral');axes[3].set_xlabel('Time (s)')
    for ax in axes:ax.legend();ax.grid(alpha=.25)
    axes[0].set_title('15°/s CRUISE: common-brake clipping offsets the turn-integrator error')
    fig.savefig(HERE/'allocation-coupling.png',dpi=150)
