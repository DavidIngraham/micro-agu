"""Independent SciPy chart fixtures, including wind, quaternion sign and heading."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
HERE=Path(__file__).resolve().parent
z=np.load(HERE.parent/'lqi-balanced-2026-10-17/designs/responsive/design.npz')
base=z['state'];p0=R.from_quat(base[3:7]);c0=R.from_quat(base[16:20]);rng=np.random.default_rng(81)
fixtures=[]
for i in range(30):
    s=base.copy();x=rng.normal(0,.1,16)*z['XS'];wind=rng.normal(0,2,3)
    e=p0.as_euler('xyz')+np.r_[x[:2],rng.uniform(-3,3)]
    p=R.from_euler('xyz',e);c=p*p0.inv()*c0*R.from_rotvec(x[2:5])
    s[3:7]=p.as_quat()*(-1 if i%2 else 1);s[16:20]=c.as_quat()*(-1 if i%3 else 1)
    s[7:10]=p.apply(p0.inv().apply(base[7:10])+x[5:8])+wind
    s[10:13]=p.apply(p0.inv().apply(base[10:13])+x[8:11])
    s[23:26]=c.apply(c0.inv().apply(base[23:26])+x[11:14]);s[2]=-x[15]
    fixtures.append(dict(state=s.tolist(),rpm=float(z['rpm']+1000*x[14]),wind=wind.tolist(),expected=(x/z['XS']).tolist()))
(HERE/'chart-fixtures.json').write_text(json.dumps(fixtures))
