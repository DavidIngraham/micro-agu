"""Independent power accounting, checked against compiled panel forces and RHS."""
from common import *
import numpy as np,json
from scipy.spatial.transform import Rotation as R
from profiles import make_simulator
import dynamics

def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def inspect(y,u,sim):
 p=dynamics.params(sim.parameters);mp=sim.geometry['mass']-p[51];mc=p[51]
 Rp=R.from_quat(y[3:7]).as_matrix();Rc=R.from_quat(y[16:20]).as_matrix()
 vp=y[7:10];wp=y[10:13];vc=y[20:23];wc=y[23:26];v=Rc.T@vc;w=Rc.T@wc
 eta=(np.arange(5)+.5)/5*2-1;theta=eta*p[38]
 pos=np.array([[0,p[2]/(2*p[38])*np.sin(a),p[2]/(2*p[38])*(1-np.cos(a))] for a in theta]);pos[:,2]-=pos[:,2].mean()
 force=np.zeros(3);moment=np.zeros(3);powers=np.zeros(3)
 for e,a,x in zip(eta,theta,pos):
  Q=R.from_rotvec([a+p[52]*(u[2]-u[1]),0,0]).as_matrix();local=Q.T@(v+np.cross(w,x));speed=np.linalg.norm(local)
  br=(u[1] if e<0 else u[2] if e>0 else (u[1]+u[2])/2)/.072
  alpha=np.arctan2(local[2],local[0]);cl=(p[4]+p[5]*alpha+p[9]*br)/(1+(abs(alpha)/.6)**4)
  cds=np.array([p[6],p[7]*cl**2,p[10]*br**2]);powers+=.5*p[0]*p[1]/5*speed**3*cds
  f=Q@(.5*p[0]*p[1]/5*(np.hypot(local[0],local[2])*cl*np.array([local[2],0,-local[0]])-speed*cds.sum()*local))
  force+=f;moment+=np.cross(x,f)
 out=np.zeros(6);dynamics.lib.panel_loads.argtypes=[dynamics.ptr,dynamics.ptr,__import__('ctypes').c_double,__import__('ctypes').c_double,dynamics.ptr,dynamics.ptr]
 dynamics.lib.panel_loads(np.ascontiguousarray(v),np.ascontiguousarray(w),u[1]/.072,u[2]/.072,p,out)
 mismatch=np.max(abs(out-np.r_[force,moment]))
 speed=np.linalg.norm(v);alpha=np.arctan2(v[2],v[0]);beta=np.arcsin(np.clip(v[1]/speed,-1,1));rates=w*np.array([p[2],p[3],p[2]])/(2*speed);diff=(u[2]-u[1])/.072
 co=np.array([p[11]*beta+p[12]*rates[0]+p[13]*diff,p[14]+p[15]*alpha+p[16]*rates[1],p[27]*beta+p[28]*rates[2]+p[29]*diff]);extra=.5*p[0]*speed**2*p[1]*np.array([p[2],p[3],p[2]])*co
 err=R.from_matrix(Rp.T@Rc).as_rotvec();dw=Rp.T@(wc-wp)
 payload_drag=.5*p[0]*p[17]*np.linalg.norm(vp)**3;joint_loss=np.sum(p[24:27]*dw**2)
 def energy(z):
  A=R.from_quat(z[3:7]).as_matrix();B=R.from_quat(z[16:20]).as_matrix();ww=A.T@z[10:13]
  Ib=np.array([[p[42],p[45],p[46]],[p[45],p[43],p[47]],[p[46],p[47],p[44]]])
  B1=np.c_[np.eye(3),-skew([0,0,p[36]])];B2=np.c_[np.eye(3),-skew([0,0,p[37]])]
  H=B1.T@np.diag([p[30],0,0])@B1+B2.T@np.diag([0,p[31],p[32]])@B2;H[3:,3:]+=np.diag(p[33:36])
  nu=np.r_[B.T@z[20:23],B.T@z[23:26]];e=R.from_matrix(A.T@B).as_rotvec()
  return .5*mp*np.dot(z[7:10],z[7:10])+.5*ww@Ib@ww+.5*mc*np.dot(z[20:23],z[20:23])+.5*np.sum(np.array([.075,.008,.077])*nu[3:]**2)+.5*nu@H@nu-9.80665*(mp*z[2]+mc*z[15])+.5*np.sum(p[21:24]*e**2)
 dy=dynamics.derivative(y,u,sim.parameters,sim.geometry['mass'],np.zeros(3));eps=1e-5;edot=(energy(y+eps*dy)-energy(y-eps*dy))/(2*eps)
 expected=-powers.sum()-payload_drag-joint_loss+extra@w
 hinge=vp+np.cross(wp,Rp@p[39:42])-vc-np.cross(wc,Rc@np.array([0,0,.85]))
 return dict(base_drag_W=powers[0],induced_drag_W=powers[1],brake_drag_W=powers[2],payload_drag_W=payload_drag,joint_damping_W=joint_loss,extra_moment_power_W=extra@w,gravity_power_W=9.80665*(mp*vp[2]+mc*vc[2]),energy_residual_W=edot-expected,panel_error=mismatch,panel_work_error_W=force@v+moment@w+powers.sum(),hinge_velocity_error=np.linalg.norm(hinge),panel_roll_Nm=moment[0],extra_roll_Nm=extra[0])

if __name__=='__main__':
 rows=[]
 for profile in ['mission','dynamic_roll']:
  sim=make_simulator('trout_lake',profile)
  for side,fraction in [('baseline',0),('left',1),('right',1)]:
   name=f'{profile}-{side}-{fraction}-1.5-1.5';a=np.load(RAW/f'{name}.npz');m=np.where((a['t']>=18)&(a['t']<=25))[0][::5]
   assert max(a['rpm'][m])<1e-3
   samples=[inspect(a['state'][i],sim.controls(*a['applied'][i]),sim) for i in m]
   row=dict(name=name,mean={k:float(np.mean([x[k] for x in samples])) for k in samples[0]},max_abs={k:float(max(abs(x[k]) for x in samples)) for k in ['energy_residual_W','panel_error','panel_work_error_W','hinge_velocity_error']});rows.append(row)
 (OUT/'power.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
