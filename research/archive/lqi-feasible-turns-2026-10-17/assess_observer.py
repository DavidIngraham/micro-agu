"""Compare observer canopy estimates against truth strictly for scoring."""
import json
import numpy as np
from scipy.spatial.transform import Rotation as R
from build import HERE
from local_analysis import arrays
def main():
    z=arrays((HERE/'runtime/AP_CustomControl_LQI_gains.h').read_text());base=np.load(HERE/'base.npz')['state'];relative=R.from_quat(base[3:7]).inv()*R.from_quat(base[16:20]);rates=R.from_quat(base[16:20]).inv().apply(base[23:26]);records=[]
    for p in sorted((HERE/'ablation').glob('*/*/00[01].json')):
        data=json.loads(p.read_text());ss=[s for s in data['samples'] if len(s.get('observer',[]))==16 and s['phase'] in ['turn1','reverse','turn2']]
        if not ss:continue
        x=np.array([s['truth'] for s in ss]);o=np.array([s['observer'] for s in ss])*z['XS'];truth=R.from_quat(x[:,3:7]).inv()*R.from_quat(x[:,16:20]);estimate=relative*R.from_rotvec(o[:,2:5]);error=(truth.inv()*estimate).as_rotvec()*180/np.pi
        rate_error=(rates+o[:,11:14]-R.from_quat(x[:,16:20]).inv().apply(x[:,23:26]))*180/np.pi
        records.append(dict(variant=p.parent.parent.name,case=data['spec']['id'],angle_rms_deg=np.sqrt(np.mean(error**2,axis=0)).tolist(),canopy_rate_rms_deg_s=np.sqrt(np.mean(rate_error**2,axis=0)).tolist()))
    (HERE/'observer-errors.json').write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
if __name__=='__main__':main()
