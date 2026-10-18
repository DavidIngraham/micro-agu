"""Align logged observer canopy states to truth; quantify feedback consequences."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'lqi-balanced-2026-10-17'

def assess(path,design):
    d=json.loads(path.read_text());z=np.load(design/'design.npz')
    t=np.array([s['time'] for s in d['samples']]);y=np.array([s['truth'] for s in d['samples']])
    td=np.array([s['time'] for s in d['diagnostics']]);a=np.array([s['values'] for s in d['diagnostics']])
    p,c=R.from_quat(y[:,3:7]),R.from_quat(y[:,16:20]);base=z['state']
    p0,c0=R.from_quat(base[3:7]),R.from_quat(base[16:20])
    angles=((p0.inv()*c0).inv()*p.inv()*c).as_rotvec()
    rates=c.inv().apply(y[:,23:26])-c0.inv().apply(base[23:26])
    truth=np.c_[angles,rates];idx=[2,3,4,11,12,13]
    truth=np.array([np.interp(td,t,truth[:,i]) for i in range(6)]).T
    error=a[:,16+np.array(idx)]-truth/z['XS'][idx]
    feedback=-(error@z['K'][:,idx].T)*z['US']
    wind=np.array([s['wind'] for s in d['samples']])
    air=p.inv().apply(y[:,7:10]-wind)-p0.inv().apply(base[7:10])
    air=np.array([np.interp(td,t,air[:,i]) for i in range(3)]).T
    measured_air_error=a[:,9:12]*z['XS'][5:8]-air
    estimated_air_error=a[:,21:24]*z['XS'][5:8]-air
    return {name:dict(angle_rms_deg=np.sqrt(np.mean((error[m,:3]*z['XS'][idx[:3]]*180/np.pi)**2,axis=0)).tolist(),
              absolute_canopy_rate_rms_deg_s=np.sqrt(np.mean((error[m,3:]*z['XS'][idx[3:]]*180/np.pi)**2,axis=0)).tolist(),
              feedback_error_rms_fraction=np.sqrt(np.mean(feedback[m]**2,axis=0)).tolist(),
              air_velocity_measurement_rms_m_s=np.sqrt(np.mean(measured_air_error[m]**2,axis=0)).tolist(),
              air_velocity_observer_rms_m_s=np.sqrt(np.mean(estimated_air_error[m]**2,axis=0)).tolist())
            for name,m in [('quiet',(td>5)&(td<20)),('climb',(td>22)&(td<35)),('descent',(td>122)&(td<135)),('updraft',(td>220)&(td<240))]}

if __name__=='__main__':
    out=assess(OLD/'training/responsive/01.json',OLD/'designs/responsive')
    (HERE/'observer-errors.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
    if (HERE/'selection.json').exists():
        name=json.loads((HERE/'selection.json').read_text())['selected']
        compared={n:assess(HERE/f'training/{n}/01.json',HERE/f'designs/{n}') for n in ['fixed',name]}
        (HERE/'observer-errors-compared.json').write_text(json.dumps(compared,indent=2))
