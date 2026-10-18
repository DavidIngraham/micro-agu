from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import gaussian_filter1d
from scipy.optimize import lsq_linear
from napkin_geometry import configuration
import fast_napkin_causal as fast
O=Path(__file__).parent;plan=json.loads((O/'long-horizon-refit-plan.json').read_text());p=json.loads((O/'curved-added-fit.json').read_text())['parameters'];groups={};summary=[];fig,axs=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
def skew(v):return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
for alpha in [.175,.35,.525]:
 XX=[];YY=[];daytag=[]
 for day,start in plan['training']:
  a=np.load(O/(day+'-causal-input.npz'));prop=np.load(O/(day+'-propulsion-9x45.npz'));i=np.searchsorted(a['t'],start);sl=slice(i,i+376);y=a['Y'][sl];g=configuration(day);rp=R.from_euler('xyz',y[:,:3]);omega=rp.apply(gaussian_filter1d(y[:,3:6],2,axis=0));angular=gaussian_filter1d(np.gradient(omega,.04,axis=0),2,axis=0);arm=rp.apply(np.broadcast_to(g['imu'],(376,3)));specific=gaussian_filter1d(a['force'][sl],2,axis=0);fcg=specific-np.cross(angular,arm)-np.cross(omega,np.cross(omega,arm));v=y[:,6:9]-np.cross(omega,arm);va=v-a['wind'][sl];bodyva=rp.inv().apply(va);Tmag=np.array([fast.lib.mapped_thrust(float(r),float(vx)) for r,vx in zip(prop['rpm'][sl],bodyva[:,0])]);T=rp.apply(np.c_[Tmag,np.zeros((376,2))]);D=-.5*1.225*.015*np.linalg.norm(va,axis=1)[:,None]*va;J=g['payload_mass']*fcg-T-D
  # Canopy orientation proxy: lift direction from total specific force, air-relative heading, fixed alpha.
  F=g['mass']*fcg-T-D;e=va/np.linalg.norm(va,axis=1)[:,None];down=-(F-e*np.sum(F*e,axis=1)[:,None]);down/=np.linalg.norm(down,axis=1)[:,None];span=np.cross(down,e);span/=np.linalg.norm(span,axis=1)[:,None];rc=R.from_matrix(np.stack([np.cos(alpha)*e-np.sin(alpha)*down,span,np.sin(alpha)*e+np.cos(alpha)*down],axis=2));rel=(rp.inv()*rc).as_rotvec();wc=np.zeros_like(omega);wc[1:-1]=(rc[2:]*rc[:-2].inv()).as_rotvec()/.08;wc[[0,-1]]=wc[[1,-2]];wc=gaussian_filter1d(wc,2,axis=0);dw=rp.inv().apply(wc-omega)
  wb=rp.inv().apply(omega);ab=rp.inv().apply(angular);needed=ab@g['inertia'].T+np.cross(wb,wb@g['inertia'].T)-np.cross(np.broadcast_to(g['thrust'],(376,3)),rp.inv().apply(T))-np.cross(np.broadcast_to(g['hinge'],(376,3)),rp.inv().apply(J));cols=[];target=[]
  for j,er in enumerate(rel):
   th=np.linalg.norm(er);E=skew(er);co=1/12 if th<1e-4 else 1/th**2-(1+np.cos(th))/(2*th*np.sin(th));Ji=np.eye(3)-.5*E+co*E@E;rollterm=(Ji.T@np.array([er[0],0,0]))[0];yawterm=(Ji.T@np.array([0,0,.05*er[2]]))[0];cols.append([rollterm,dw[j,0],Tmag[j]]);target.append(needed[j,0]-yawterm)
  XX.extend(np.array(cols)[10:-10:5]);YY.extend(np.array(target)[10:-10:5]);daytag.extend([day]*len(np.array(target)[10:-10:5]))
 X=np.array(XX);Y=np.array(YY);f=lsq_linear(X,Y,bounds=([0,0,-.10],[10,2,.10]));pred=X@f.x;old=X@np.array([p['joint_k_roll'],p['joint_c_roll'],-.02]);row=dict(alpha_deg=float(np.rad2deg(alpha)),roll_stiffness_Nm_rad=float(f.x[0]),roll_damping_Nm_s_rad=float(f.x[1]),prop_torque_per_thrust_m=float(f.x[2]),fitted_RMSE_Nm=float(np.sqrt(np.mean((Y-pred)**2))),existing_RMSE_Nm=float(np.sqrt(np.mean((Y-old)**2))),samples=len(Y));summary.append(row)
 if alpha==.35:
  for col,day in enumerate(['2026-01-03','2026-02-15']):
   m=np.array(daytag)==day;axs[0,col].scatter(Y[m],old[m],s=7,alpha=.5,label='Existing');axs[0,col].scatter(Y[m],pred[m],s=7,alpha=.5,label='Inverse fit');low=min(Y[m].min(),old[m].min());high=max(Y[m].max(),old[m].max());axs[0,col].plot([low,high],[low,high],'k:');axs[0,col].set(title='Hood River' if col==0 else 'Trout Lake',xlabel='Inferred required torque (Nm)',ylabel='Modeled torque (Nm)');axs[0,col].legend();axs[1,col].scatter(X[m,0],Y[m],s=7,alpha=.5);axs[1,col].set(xlabel='Relative roll spring factor (rad)',ylabel='Required roll torque (Nm)')
for ax in axs.flat:ax.grid(alpha=.25)
fig.suptitle('Payload inverse-torque diagnostic — inferred canopy orientation, not a measurement');fig.savefig(O/'payload-inverse-torque.png',dpi=150);(O/'payload-inverse-torque.json').write_text(json.dumps(dict(scope='Offline inverse dynamics on training blocks only. Canopy orientation is a force/velocity proxy; alpha sensitivity, unmeasured relative canopy acceleration and thrust reaction torque confound this estimate. No direct parameter replacement.',rows=summary),indent=2));print(json.dumps(summary,indent=2))
