"""AI-assisted dimensional acceptance gates, fixed before observing results."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def longest(mask,t):
    best=0.;start=None
    for good,tt in zip(mask,t):
        if good:
            if start is None:start=tt
            best=max(best,tt-start)
        else:start=None
    return float(best)

def assess(path):
    d=json.loads(Path(path).read_text());s=d['spec'];rows=d['samples'];reasons=[]
    r=dict(id=s['id'],rate=s['rate'],scheme=s['scheme'],complete=d['complete'],failure=d['failure'])
    if not d['complete'] or not rows:return dict(r,passed=False,reasons=[d['failure'] or 'Incomplete'])
    t=np.array([a['time'] for a in rows]);x=np.array([a['truth'] for a in rows]);wind=np.array([a['wind'] for a in rows]);cmd=np.array([a['command'] for a in rows]);br=np.array([a['brakes'] for a in rows])/(np.array([1300,1500])*np.pi*55/360*.1)
    rp=Rotation.from_quat(x[:,3:7]);euler=rp.as_euler('xyz');body=rp.inv().apply(x[:,10:13]);rate=np.rad2deg((body[:,1]*np.sin(euler[:,0])+body[:,2]*np.cos(euler[:,0]))/np.cos(euler[:,1]))
    relative=np.rad2deg(rp.inv().apply(x[:,23:26]-x[:,10:13])[:,1]);speed=np.linalg.norm(x[:,7:10]-wind,axis=1);height=-x[:,2];ph=np.array([a['phase'] for a in rows]);phases={}
    if np.max(np.diff(t))>.5:reasons.append('Sampling gap')
    for name,sign in [('turn1',s['sign']),('reverse',-s['sign']),('turn2',s['sign'])]:
        m=ph==name;tt=t[m];late=m&(t>tt[0]+20);goal=sign*s['rate'];error=rate[late]-goal
        stats=dict(mean_rate=float(np.mean(rate[late])),mean_error=float(abs(np.mean(error))),p90_error=float(np.percentile(abs(error),90)),rate_std=float(np.std(rate[late])),alt_rms=float(np.sqrt(np.mean((height[m]-60)**2))),alt_peak=float(np.max(abs(height[m]-60))),relative_pitch_rate_peak=float(np.max(abs(relative[m]))),relative_pitch_rate_late_rms=float(np.sqrt(np.mean(relative[late]**2))),speed_p05=float(np.percentile(speed[m],5)),speed_min=float(np.min(speed[m])),full_brake_longest=longest(np.any(br[m]>.99,axis=1),tt),throttle_ceiling_fraction=float(np.mean(cmd[m,0]>=.594)))
        phases[name]=stats
        checks={'mean turn tracking':stats['mean_error']<=max(2,.1*s['rate']),'p90 turn tracking':stats['p90_error']<=max(3,.2*s['rate']),'turn oscillation':stats['rate_std']<=max(3,.15*s['rate']),'altitude RMS':stats['alt_rms']<=3,'altitude peak':stats['alt_peak']<=8,'relative pitch peak':stats['relative_pitch_rate_peak']<=30,'relative pitch late RMS':stats['relative_pitch_rate_late_rms']<=5,'airspeed p05':stats['speed_p05']>=3,'airspeed minimum':stats['speed_min']>=2,'full brake duration':stats['full_brake_longest']<=5}
        reasons += [name+': '+k for k,v in checks.items() if not v]
    for name in ['recover1','recover2']:
        m=ph==name;tt=t[m];late=m&(t>tt[-1]-10)
        stats=dict(rate_rms=float(np.sqrt(np.mean(rate[late]**2))),alt_rms=float(np.sqrt(np.mean((height[late]-60)**2))),relative_pitch_rate_rms=float(np.sqrt(np.mean(relative[late]**2))))
        phases[name]=stats
        if stats['rate_rms']>3:reasons.append(name+': heading recovery')
        if stats['alt_rms']>1.5:reasons.append(name+': altitude recovery')
        if stats['relative_pitch_rate_rms']>3:reasons.append(name+': oscillation recovery')
    diag=[a['diagnostic'] for a in rows if len(a['diagnostic'])==15]
    if s['scheme']=='lqi':
        if len(diag)<.9*len(rows):reasons.append('Missing controller diagnostics')
        elif max(abs(np.array(diag)[:,14]-s['rate']))>.01:reasons.append('Rate parameter mismatch')
    r.update(passed=not reasons,reasons=reasons,phases=phases,max_roll=float(np.max(abs(np.rad2deg(euler[:,0])))),max_pitch=float(np.max(abs(np.rad2deg(euler[:,1])))))
    return r
