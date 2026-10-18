"""Two rigid bodies joined at one point. SI units, inertial NED vectors.
State per body: position(3), quaternion xyzw(4), velocity(3), omega_world(3).
No aerodynamic assumptions; constraint reaction is solved, never integrated as a spring.
"""
from dataclasses import dataclass
import numpy as np
from scipy.spatial.transform import Rotation

def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])

def quat_rate(q,w):
 v=q[:3];s=q[3];return .5*np.r_[s*w+np.cross(w,v),-w@v]

@dataclass
class Body:
 mass:float
 inertia:np.ndarray
 hinge:np.ndarray
 def __post_init__(self):
  self.inertia=np.asarray(self.inertia,float);self.hinge=np.asarray(self.hinge,float)
  if self.mass<=0 or not np.allclose(self.inertia,self.inertia.T) or np.min(np.linalg.eigvalsh(self.inertia))<=0:raise ValueError('Positive mass and symmetric positive definite inertia required')

class TwoBody:
 def __init__(self,payload,canopy,locked=False):self.bodies=[payload,canopy];self.locked=locked
 def quantities(self,y):
  out=[]
  for i,b in enumerate(self.bodies):
   s=y[i*13:(i+1)*13];R=Rotation.from_quat(s[3:7]).as_matrix();r=R@b.hinge;I=R@b.inertia@R.T
   out.append((s[:3],R,r,I,s[7:10],s[10:13]))
  return out
 def accelerations(self,y,forces,moments,stabilization=0):
  q=self.quantities(y);M=np.zeros((12,12));f=np.zeros(12)
  for i,(b,(_,R,r,I,v,w)) in enumerate(zip(self.bodies,q)):
   k=6*i;M[k:k+3,k:k+3]=np.eye(3)*b.mass;M[k+3:k+6,k+3:k+6]=I;f[k:k+3]=forces[i];f[k+3:k+6]=moments[i]-np.cross(w,I@w)
  xp,Rp,rp,Ip,vp,wp=q[0];xc,Rc,rc,Ic,vc,wc=q[1]
  G=np.hstack([np.eye(3),-skew(rp),-np.eye(3),skew(rc)])
  bias=-np.cross(wp,np.cross(wp,rp))+np.cross(wc,np.cross(wc,rc))
  gap=xp+rp-xc-rc;vgap=vp+np.cross(wp,rp)-vc-np.cross(wc,rc)
  if stabilization:bias-=2*stabilization*vgap+stabilization**2*gap
  if self.locked:
   G=np.vstack([G,np.hstack([np.zeros((3,3)),np.eye(3),np.zeros((3,3)),-np.eye(3)])]);bias=np.r_[bias,np.zeros(3)]
  n=len(bias);A=np.block([[M,-G.T],[G,np.zeros((n,n))]]);sol=np.linalg.solve(A,np.r_[f,bias]);return sol[:12].reshape(2,6),sol[12:],dict(gap=gap,velocity_gap=vgap,constraint_residual=G@sol[:12]-bias)
 def derivative(self,t,y,loads,stabilization=0):
  F,T=loads(t,y,self.quantities(y));a,_,_=self.accelerations(y,F,T,stabilization);dy=np.zeros(26)
  for i in range(2):
   s=y[13*i:13*(i+1)];k=13*i;dy[k:k+3]=s[7:10];dy[k+3:k+7]=quat_rate(s[3:7],s[10:13]);dy[k+7:k+10]=a[i,:3];dy[k+10:k+13]=a[i,3:]
  return dy
 def invariants(self,y):
  P=np.zeros(3);H=np.zeros(3);E=0.
  for b,(x,R,r,I,v,w) in zip(self.bodies,self.quantities(y)):
   P+=b.mass*v;H+=I@w+np.cross(x,b.mass*v);E+=.5*b.mass*(v@v)+.5*w@I@w
  return P,H,E

def initial_state(model,Rp=None,Rc=None,v=None,wp=None,wc=None):
 Rp=np.eye(3) if Rp is None else Rp;Rc=Rp if Rc is None else Rc;v=np.zeros(3) if v is None else np.asarray(v);wp=np.zeros(3) if wp is None else np.asarray(wp);wc=wp if wc is None else np.asarray(wc)
 rp=Rp@model.bodies[0].hinge;rc=Rc@model.bodies[1].hinge;xc=rp-rc;vc=v+np.cross(wp,rp)-np.cross(wc,rc)
 return np.r_[np.zeros(3),Rotation.from_matrix(Rp).as_quat(),v,wp,xc,Rotation.from_matrix(Rc).as_quat(),vc,wc]
