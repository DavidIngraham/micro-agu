"""Research curved-canopy panel loads; geometry and coefficients provisional."""
import numpy as np
from scipy.spatial.transform import Rotation

def loads(v,omega,left,right,curvature=.6,span=2.15,area=1.16,CL0=.4,CLa=2.,CD0=.15,kdrag=.3,CLbrake=1.,CDbrake=1.,rho=1.225,n=5):
 eta=(np.arange(n)+.5)/n*2-1;theta=eta*curvature
 if abs(curvature)<1e-7:pos=np.column_stack([eta*0,eta*span/2,eta*0])
 else:pos=np.column_stack([eta*0,span/(2*curvature)*np.sin(theta),span/(2*curvature)*(1-np.cos(theta))]);pos[:,2]-=pos[:,2].mean()
 forces=[];moments=[];lift_work=[];drag_work=[]
 for e,t,r in zip(eta,theta,pos):
  R=Rotation.from_euler('x',t).as_matrix();local=R.T@(np.asarray(v)+np.cross(omega,r));speed=np.linalg.norm(local);xz=np.hypot(local[0],local[2]);br=left if e<0 else right if e>0 else (left+right)/2
  alpha=np.arctan2(local[2],local[0]);cl=(CL0+CLa*alpha+CLbrake*br)/(1+(abs(alpha)/.6)**4);cd=CD0+kdrag*cl*cl+CDbrake*br*br
  lift=.5*rho*area/n*xz*cl*np.array([local[2],0,-local[0]]);drag=-.5*rho*area/n*speed*cd*local;f=R@(lift+drag);forces.append(f);moments.append(np.cross(r,f));lift_work.append(float(lift@local));drag_work.append(float(drag@local))
 return np.sum(forces,axis=0),np.sum(moments,axis=0),dict(positions=pos,forces=np.array(forces),lift_work=lift_work,drag_work=drag_work)

if __name__=='__main__':
 from pathlib import Path
 import json
 O=Path(__file__).parent
 F,M,_=loads([5,0,0],[0,0,0],.2,.2,curvature=0);cl=.6;expected=.5*1.225*25*1.16*np.array([-(.15+.3*cl*cl+.04),0,-cl]);assert np.max(abs(F-expected))<1e-12;assert np.max(abs(M))<1e-12
 flat_error=float(np.max(abs(F-expected)));rng=np.random.default_rng(6);mirror=[];works=[]
 for i in range(100):
  v=np.array([rng.uniform(2,9),rng.uniform(-2,2),rng.uniform(-1,2)]);w=rng.normal(size=3);l,r=rng.uniform(0,1,2);F,M,d=loads(v,w,l,r);G,N,_=loads(v*[1,-1,1],w*[-1,1,-1],r,l);mirror.append(max(np.max(abs(G-F*[1,-1,1])),np.max(abs(N-M*[-1,1,-1]))));works+=d['lift_work'];assert max(d['drag_work'])<=1e-12
 assert max(mirror)<1e-10;assert max(abs(np.array(works)))<1e-10
 result=dict(flat_uniform_force_error=flat_error,mirror_error=max(mirror),max_lift_power_abs=max(abs(np.array(works))),cases=100,scope='Load-level symmetry, lift orthogonality, nonpositive drag work, and flat uniform-flow analytic check. No flight fit.')
 (O/'curved-panel-load-checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 fig,axes=plt.subplots(1,2,figsize=(11,4))
 angles=np.linspace(0,1.1,60);f=[];m=[]
 for ang in angles:
  F,M,_=loads([5,0,0],[0,0,0],0,.4,curvature=ang);f.append(F);m.append(M)
 f=np.array(f);m=np.array(m)
 axes[0].plot(np.rad2deg(angles),f[:,1]);axes[0].set(xlabel='Canopy arc half-angle (deg)',ylabel='Lateral force (N)',title='Right brake: lateral force changes with curvature')
 axes[1].plot(np.rad2deg(angles),m[:,0],label='Roll');axes[1].plot(np.rad2deg(angles),m[:,2],label='Yaw');axes[1].set(xlabel='Canopy arc half-angle (deg)',ylabel='Moment (N m)',title='Moments from panel forces');axes[1].legend()
 for ax in axes:ax.grid(alpha=.2)
 fig.suptitle('Illustrative five-panel load model—not fitted to flight data');fig.tight_layout();fig.savefig(O/'iteration-curved-panel-loads.png',dpi=150)

