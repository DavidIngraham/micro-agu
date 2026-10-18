import json
import numpy as np
from scipy.spatial.transform import Rotation
from build import HERE
for p in sorted((HERE/'deployment/qualification').glob('transition-*/*.json')):
    if '-spec' in p.name:continue
    d=json.loads(p.read_text());s=d['samples'];x=np.array([a['truth'] for a in s]);r=Rotation.from_quat(x[:,3:7]);q=np.rad2deg(r.inv().apply(x[:,23:26]-x[:,10:13])[:,1]);i=int(np.argmax(abs(q)));a=s[i]
    print(str(p.relative_to(HERE)),d['complete'],round(a['time'],2),a['phase'],round(q[i],2),'cmd',a['command'])
