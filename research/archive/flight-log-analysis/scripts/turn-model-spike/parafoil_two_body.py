"""Research parafoil aerodynamic loads for the verified two-body solver.
Coefficients provisional. No production integration. Brake controls are line pull in metres.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from two_body import Body,TwoBody,initial_state
DEFAULT=dict(rho=1.225,S=1.16,b=2.15,c=.54,CL0=.4,CLa=2.,CD0=.15,k_drag=.3,CYbeta=-.2,CLbrake=1.,CDbrake=1.,Clbeta=-.03,Clp=-.1,Cldelta=.05,Cm0=.02,Cma=-.2,Cmq=-2.,Cnbeta=.03,Cnr=-.05,Cndelta=.04,payload_CDA=.015,thrust_max=10.,thrust_z=-.10,prop_torque=-.02,joint_k_roll=.4,joint_k_pitch=0.,joint_k_yaw=.05,joint_c_roll=.03,joint_c_pitch=.015,joint_c_yaw=.02)

def aero(v,R,omega,brakes,p):
 vb=R.T@v;V=np.linalg.norm(vb)
 if V<.1:return np.zeros(3),np.zeros(3),dict(alpha=0.,beta=0.,lift=0.,drag=0.)
 alpha=np.arctan2(vb[2],vb[0]);beta=np.arcsin(np.clip(vb[1]/V,-1,1));q=.5*p['rho']*V*V;left,right=brakes;mean=.5*(left+right)/.072;diff=(right-left)/.072
 # Bounded smooth lift extension for exploratory large-angle runs, not calibrated stall data.
 CL=(p['CL0']+p['CLa']*alpha+p['CLbrake']*mean)/(1+(abs(alpha)/.6)**4)
 CD=p['CD0']+p['k_drag']*CL*CL+p['CDbrake']*mean**2
 e=vb/V;span=np.array([0.,1,0]);down=np.cross(e,span);nd=np.linalg.norm(down)
 if nd<1e-8:down=np.array([0.,0,1.]);down-=e*(e@down);nd=np.linalg.norm(down)
 down/=nd;side=np.cross(down,e)
 Fbody=q*p['S']*(-CD*e-CL*down+p['CYbeta']*beta*side)
 w=R.T@omega;rates=w*np.array([p['b'],p['c'],p['b']])/(2*V)
 C=np.array([p['Clbeta']*beta+p['Clp']*rates[0]+p['Cldelta']*diff,p['Cm0']+p['Cma']*alpha+p['Cmq']*rates[1],p['Cnbeta']*beta+p['Cnr']*rates[2]+p['Cndelta']*diff])
 Mbody=q*p['S']*np.array([p['b'],p['c'],p['b']])*C
 return R@Fbody,R@Mbody,dict(alpha=alpha,beta=beta,lift=q*p['S']*CL,drag=q*p['S']*CD)

class Parafoil(TwoBody):
 def __init__(self,mass=1.385,params=None,locked=False):
  # Geometry/inertia are provisional and must be bounded using CAD and measurements.
  super().__init__(Body(mass-.19,np.diag([.006,.008,.003]),[0,0,-.13]),Body(.19,np.diag([.075,.008,.077]),[0,0,.85]),locked)
  self.params=DEFAULT.copy();self.params.update(params or {})
 def loads(self,y,u,wind=np.zeros(3)):
  q=self.quantities(y);p=self.params;F=np.zeros((2,3));M=np.zeros((2,3));g=np.array([0.,0,9.80665]);xp,Rp,rp,Ip,vp,wp=q[0];xc,Rc,rc,Ic,vc,wc=q[1]
  T=Rp@np.array([p['thrust_max']*u[0],0,0]);v=vp-wind;D=-.5*p['rho']*p['payload_CDA']*np.linalg.norm(v)*v
  F[0]=self.bodies[0].mass*g+T+D;M[0]=np.cross(Rp@np.array([0,0,p['thrust_z']]),T)+Rp@np.array([p['prop_torque']*p['thrust_max']*u[0],0,0])
  A,M[1],info=aero(vc-wind,Rc,wc,u[1:],p);F[1]=self.bodies[1].mass*g+A
  # Relative orientation from payload to canopy, expressed in payload axes.
  error=Rotation.from_matrix(Rp.T@Rc).as_rotvec();dw=Rp.T@(wc-wp)
  k=np.array([p['joint_k_roll'],p['joint_k_pitch'],p['joint_k_yaw']]);c=np.array([p['joint_c_roll'],p['joint_c_pitch'],p['joint_c_yaw']]);joint=Rp@(k*error+c*dw)
  M[0]+=joint;M[1]-=joint
  return F,M
