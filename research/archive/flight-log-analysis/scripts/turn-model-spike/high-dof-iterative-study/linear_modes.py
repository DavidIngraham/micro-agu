from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent

def perturb(base,x,g):
 y=base.copy();rp=Rot.from_quat(base[3:7])*Rot.from_rotvec(x[:3]);rc=Rot.from_quat(base[16:20])*Rot.from_rotvec(x[3:6]);y[3:7]=rp.as_quat();y[16:20]=rc.as_quat();y[7:10]+=Rot.from_quat(base[3:7]).apply(x[6:9]);y[10:13]+=Rot.from_quat(base[3:7]).apply(x[9:12]);y[23:26]+=Rot.from_quat(base[16:20]).apply(x[12:15]);ap=rp.apply(g['hinge']);ac=rc.apply([0,0,.85]);y[13:16]=y[:3]+ap-ac;y[20:23]=y[7:10]+np.cross(y[10:13],ap)-np.cross(y[23:26],ac);return y

def difference(base,y):
 rp=Rot.from_quat(base[3:7]);rc=Rot.from_quat(base[16:20]);return np.r_[(rp.inv()*Rot.from_quat(y[3:7])).as_rotvec(),(rc.inv()*Rot.from_quat(y[16:20])).as_rotvec(),rp.inv().apply(y[7:10]-base[7:10]),rp.inv().apply(y[10:13]-base[10:13]),rc.inv().apply(y[23:26]-base[23:26])]

def linearize(sim,base,rpm,control,dt=.002,eps=1e-5):
 nextbase,_=sim.step(base,rpm,control,dt=dt);jac=np.zeros((15,15))
 for j in range(15):
  x=np.zeros(15);x[j]=eps;yp,_=sim.step(perturb(base,x,sim.geometry),rpm,control,dt=dt);ym,_=sim.step(perturb(base,-x,sim.geometry),rpm,control,dt=dt);jac[:,j]=(difference(nextbase,yp)-difference(nextbase,ym))/(2*eps)
 eig,vec=np.linalg.eig(jac);poles=np.log(eig.astype(complex))/dt;rows=[]
 for k,p in enumerate(poles):
  if p.imag<=.01:continue
  v=vec[:,k];bodyomega=Rot.from_quat(base[3:7]).inv().apply(base[10:13]);observed=v[9:12]+np.cross(bodyomega,v[:3]);phase=np.angle(np.conj(observed[1])*observed[2],deg=True);rows.append(dict(frequency_hz=float(p.imag/(2*np.pi)),decay_per_s=float(-p.real),damping_ratio=float(-p.real/abs(p)),pitch_yaw_phase_deg=float(phase),payload_rate_mode_abs=abs(observed).tolist(),payload_rate_mode_complex=np.c_[observed.real,observed.imag].tolist(),payload_rotation_mode_abs=abs(v[:3]).tolist()))
 return sorted(rows,key=lambda r:r['frequency_hz'])
if __name__=='__main__':
 sys.path.insert(0,str(R/'candidate-v1'));from simulator import Simulator
 rows=[]
 for flight in ['hood_river','trout_lake']:
  for inertia in [0.,6.532245e-5,9e-5,1.306449e-4]:
   for yaw in ['constant','mass_linear']:
    sim=Simulator(flight,left_slack_mm=0,yaw=yaw,gyro_inertia=inertia);u=[.4,0,0];y,r=sim.replay(sim.initial_state(),np.tile(u,(3001,1)));modes=linearize(sim,y[-1],r[-1],u);row=dict(flight=flight,inertia=inertia,yaw=yaw,rpm=float(r[-1]),modes=modes);rows.append(row);print(row,flush=True)
 (R/'linear-modes.json').write_text(json.dumps(rows,indent=2))



