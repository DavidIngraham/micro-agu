"""Merge retained experiments and compute article tables with explicit windows."""
import json
import numpy as np
from study import HERE, TRACES, DT, XS, US, plants, save
from analyze import lqi
from nonlinear import plot

def main():
    rows=[]
    for name in ['baseline','candidate','effort','oracle','oracle_selected']:
        path=HERE/f'nonlinear-summary-{name}.json'
        if path.exists():rows.extend(json.loads(path.read_text()))
    save('nonlinear-summary.json',rows)
    selected=json.loads((HERE/'oracle-selected.json').read_text());p=plants()[0]
    F,G,W,U,K=lqi(p,**selected);x=np.zeros(20)
    d=np.load(TRACES/'level-small_ramp-oracle_selected.npz')['trace'];expect=[]
    for r in d:
        expect.append(np.r_[p['C']@(XS*x[:16]),U@x])
        x=F@x+G@r[[9,10]]
    expect=np.array(expect)[:,[5,0,1,6,7]];expect[:,2]*=180/np.pi
    actual=d[:,[1,2,4,7,8]].copy();actual[:,3]-=p['command'][0];actual[:,4]-=p['command'][2]/(np.pi*55/360*.1*1500)
    error=np.linalg.norm(actual-expect,axis=0)/np.maximum(np.linalg.norm(expect,axis=0),1e-10)
    validation=json.loads((HERE/'linear-validation.json').read_text())
    validation['rows']=[r for r in validation['rows'] if r['controller']!='oracle_selected']
    validation['rows'].append(dict(controller='oracle_selected',relative_l2_errors=error.tolist()))
    save('linear-validation.json',validation)
    maneuver=[]
    for name in ['baseline','candidate','oracle_selected']:
        d=np.load(TRACES/f'level-maneuver-{name}.npz')['trace'];use=(d[:,0]>=5)&(d[:,0]<80);turn=(d[:,0]>=20)&(d[:,0]<60)
        maneuver.append(dict(controller=name,height_iae_m_s=float(DT*sum(abs(d[use,1]-d[use,9]))),
                             pitch_rate_rms=float(np.sqrt(np.mean(d[use,4]**2))),peak_pitch_rate=float(max(abs(d[use,4]))),
                             heading_rmse=float(np.sqrt(np.mean((d[turn,6]-d[turn,10])**2)))))
    save('article-tables.json',dict(gust=[r for r in rows if r['plant']=='level' and r['scenario']=='gust_holdout'],maneuver=maneuver,
                                   windows='Gust: 15 <= t < 80 s. Maneuver height/rate: 5 <= t < 80 s; heading: 20 <= t < 60 s.'))
    changes=[]
    for b in rows:
        if b['controller']=='baseline' and 'gust' in b['scenario']:
            c=next(r for r in rows if (r['plant'],r['scenario'],r['controller'])==(b['plant'],b['scenario'],'candidate'))
            changes.append(dict(plant=b['plant'],scenario=b['scenario'],
                **{k:round(100*(c[k]/b[k]-1),2) for k in ['height_rmse','pitch_rate_rms','brake_std','throttle_std']}))
    save('nonlinear-paired-changes.json',changes)
    plot();print('oracle linear validation',error);print(json.dumps(maneuver,indent=2))

if __name__=='__main__':main()
