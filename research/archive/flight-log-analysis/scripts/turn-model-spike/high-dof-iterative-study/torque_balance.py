from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));from run_sustained_envelope import p,bg
import fast_rotor_torque as fast
fig,axs=plt.subplots(3,3,figsize=(15,11));out=[]
for col,k in enumerate([0.,.4,p['joint_k_roll']]):
 day='2026-01-03';fname=f'roll-{day}-{0 if k==0 else k}-0-0-1.npz';d=np.load(R/fname);y=d['state'];rpm=d['rpm'];g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05));mp=g['payload_mass'];t=np.arange(len(y))*.04;rp=Rot.from_quat(y[:,3:7]);rc=Rot.from_quat(y[:,16:20]);vb=rp.inv().apply(y[:,7:10]);T=np.array([fast.lib.mapped_thrust(float(n),float(v)) for n,v in zip(rpm,vb[:,0])]);Q=np.array([fast.lib.mapped_torque(float(n),float(v)) for n,v in zip(rpm,vb[:,0])]);drag=-.5*1.225*fast.DEFAULT['payload_CDA']*np.linalg.norm(y[:,7:10],axis=1)[:,None]*y[:,7:10];Fp=rp.apply(np.c_[T,np.zeros((len(t),2))])+drag+[0,0,mp*9.80665];acc=np.gradient(y[:,7:10],.04,axis=0);J=mp*acc-Fp;hinge=np.cross(np.broadcast_to(g['hinge'],J.shape),rp.inv().apply(J));err=(rp.inv()*rc).as_rotvec();dw=rp.inv().apply(y[:,23:26]-y[:,10:13]);K=np.array([k,0,.5]);C=np.array([p['joint_c_roll'],.015,.02]);spring=[]
 for e in err:
  a=np.linalg.norm(e);E=np.array([[0,-e[2],e[1]],[e[2],0,-e[0]],[-e[1],e[0],0]]);coef=1/12+a*a/720 if a<1e-4 else 1/(a*a)-(1+np.cos(a))/(2*a*np.sin(a));ji=np.eye(3)-.5*E+coef*E@E;spring.append(ji.T@(K*e))
 spring=np.array(spring);joint=spring+C*dw;omega=rp.inv().apply(y[:,10:13]);alpha=rp.inv().apply(np.gradient(y[:,10:13],.04,axis=0));I=g['inertia'];lhs=alpha@I.T+np.cross(omega,omega@I.T);res=lhs[:,0]-(-Q+joint[:,0]+hinge[:,0]);chi=np.unwrap(np.arctan2(y[:,8],y[:,7]));q=np.rad2deg(np.gradient(chi,.04));
 axs[0,col].plot(t,np.rad2deg(rp.as_euler('xyz')[:,0]),label='payload');axs[0,col].plot(t,np.rad2deg(rc.as_euler('xyz')[:,0]),label='canopy');axs[0,col].set_title(f'Roll stiffness {k:.3g}');axs[0,col].set_ylabel('Bank deg');axs[1,col].plot(t,q,label='course rate');axs[1,col].set_ylabel('deg/s')
 for arr,label in [(-Q,'propeller'),(joint[:,0],'joint spring+damper'),(hinge[:,0],'hinge force moment')]:axs[2,col].plot(t,arr,label=label)
 axs[2,col].set_ylabel('Payload roll torque N m');axs[2,col].set_xlabel('Seconds')
 for ax in axs[:,col]:ax.grid(alpha=.2);ax.legend(fontsize=8)
 out.append(dict(k_roll=k,last10_torque_balance_residual_rms=float(np.sqrt(np.mean(res[-251:-1]**2))),last10_torque_mean=dict(propeller=float(-Q[-251:].mean()),joint=float(joint[-251:,0].mean()),hinge=float(hinge[-251:,0].mean())),last10_course_std=float(q[-251:].std())))
fig.tight_layout();fig.savefig(R/'roll-torque-balance.png',dpi=140);(R/'roll-torque-balance.json').write_text(json.dumps(dict(note='Hinge force reconstructed from finite-difference acceleration and external forces; numerical residual exposed. All axes payload FRD. No gyro term because these runs have rotor spin inertia0.',rows=out),indent=2));print(json.dumps(out,indent=2))
