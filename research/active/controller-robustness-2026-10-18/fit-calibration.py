import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
P=Path(__file__).resolve().parent
def response(rows,hz):
 a=np.array(rows);t=a[:,0];y=a[:,1:];mask=(t>=25)&(t<=45)
 r=Rotation.from_quat(y[:,3:7]).inv().apply(y[:,10:13])*180/np.pi
 x=np.column_stack([np.sin(2*np.pi*hz*t[mask]),np.cos(2*np.pi*hz*t[mask]),np.ones(sum(mask))])
 c=np.linalg.lstsq(x,r[mask,1:3],rcond=None)[0]
 return c[0]+1j*c[1]
if __name__=='__main__':
 basis=json.loads((P/'calibration-basis.json').read_text())['rows'];out=[]
 for hz in [1,1.6,2.1,3]:
  b=[next(r for r in basis if r['hz']==hz and r['axis']==axis) for axis in ['pitch','yaw']]
  h=np.column_stack([response(r['rows'],hz)/.005 for r in b])
  for phase in [-np.pi/2,np.pi/2]:
   torque=np.linalg.solve(h,np.sqrt(2)*np.array([8*np.exp(1j*phase),20]))
   out.append(dict(kind='forcing',hz=hz,amplitude=abs(torque[1]),ratio=abs(torque[0]/torque[1]),phase=float(np.angle(torque[0]/torque[1])),damping=.1,gain=0.,target_ratio=.4,target_phase=phase,target_yaw_rms=20))
 saved=json.loads((P/'forcing-calibration.json').read_text())
 error=max(abs(a[k]-b[k]) for a,b in zip(out,saved) for k in ['amplitude','ratio','phase'])
 print('Saved calibration maximum reconstruction difference',error)
 assert error<1e-4 # Frozen campaign calibration; tiny basis cross-input and window sensitivity.
