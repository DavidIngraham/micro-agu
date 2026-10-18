from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O))
from run_sustained_envelope import p,bg
import fast_rotor_torque as fast
variants={'combined':{},'half_brake_lift':dict(CLbrake=p['CLbrake']*.5),'double_brake_drag':dict(CDbrake=p['CDbrake']*2)}
def initialize(a,i,g,wind,rpm):
 obs=a['Y'][i];rp=Rot.from_euler('xyz',obs[:3]);wp=rp.apply(obs[3:6]);v=obs[6:9]-np.cross(wp,rp.apply(g['imu']));e=v-wind;e=e/np.linalg.norm(e)
 specific=a['force'][max(0,i-10):i+1].mean(axis=0);force=g['mass']*specific-rp.apply([fast.lib.mapped_thrust(rpm,float(rp.inv().apply(v-wind)[0])),0,0]);down=-(force-e*(force@e));down/=np.linalg.norm(down);span=np.cross(down,e);span/=np.linalg.norm(span);alpha=.35
 rc=Rot.from_matrix(np.column_stack([np.cos(alpha)*e-np.sin(alpha)*down,span,np.sin(alpha)*e+np.cos(alpha)*down]));pr=rp.apply(g['hinge']);cr=rc.apply([0,0,.85]);return np.r_[np.zeros(3),rp.as_quat(),v,wp,pr-cr,rc.as_quat(),v+np.cross(wp,pr)-np.cross(wp,cr),wp]
rows=[]
for day,starts in [('2026-01-03',[300,480,720,930]),('2026-02-15',[840,1920,2790,2910])]:
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05))
 fig,axs=plt.subplots(4,3,figsize=(15,13))
 for si,start in enumerate(starts):
  i=np.searchsorted(a['t'],start);sl=slice(i,i+751);truth=a['Y'][sl];t=np.arange(len(truth))*.04;wind=a['wind'][i];rpm=float(prop['rpm'][i]);voltage=float(prop['voltage'][i]);z=initialize(a,i,g,wind,rpm)
  u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]/1000]);u[:,1]=np.maximum(u[:,1]-p['left_offset'],0);u[:,2]=np.maximum(u[:,2]-p['right_offset'],0)
  ct=np.unwrap(np.arctan2(truth[:,7],truth[:,6]));ct-=ct[0];bt=np.rad2deg(truth[:,0]);vt=np.linalg.norm(truth[:,6:8],axis=1)
  for j,arr in enumerate([np.rad2deg(ct),bt,-truth[:,8]]):axs[si,j].plot(t,arr,'k',lw=2,label='flight')
  for name,change in variants.items():
   par=p|g['parameters']|dict(joint_k_yaw=.5,rotor_spin_inertia=0.,rotor_spin_sign=1.,canopy_mass=.19,Clbeta=-.03,Cnbeta=.0075,Cnr=-.0125)|change
   try:
    y,r=fast.causal_replay(z,u,par,g['mass'],wind,voltage,rpm,substeps=32);rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(g['imu'],(len(y),3))));c=np.unwrap(np.arctan2(v[:,1],v[:,0]));c-=c[0];bank=rot.as_euler('xyz',degrees=True)[:,0];ce=np.rad2deg(c-ct)
    row=dict(day=day,start=start,variant=name,course_E_deg_s=float(2/30**2*np.trapz(abs(ce),t)),course_final_error_deg=float(ce[-1]),bank_rmse_deg=float(np.sqrt(np.mean((bank-bt)**2))),speed_rmse=float(np.sqrt(np.mean((np.linalg.norm(v[:,:2],axis=1)-vt)**2))),climb_rmse=float(np.sqrt(np.mean((v[:,2]-truth[:,8])**2))),climb_bias=float(np.mean(-v[:,2]+truth[:,8])),status='finite')
    np.savez_compressed(R/f'brake-{day}-{start}-{name}.npz',state=y,rpm=r,truth=truth,controls=u,t=t,wind=wind,initial=z)
    for j,arr in enumerate([np.rad2deg(c),bank,-v[:,2]]):axs[si,j].plot(t,arr,label=name)
   except (FloatingPointError,ValueError) as e:row=dict(day=day,start=start,variant=name,status='diverged',error=str(e))
   rows.append(row)
  for j,label in enumerate(['Relative course (deg)','Payload bank (deg)','Climb (m/s)']):axs[si,j].set(title=f'{start}s / {label}',xlabel='Time since start (s)');axs[si,j].grid(alpha=.2)
  print(day,start,flush=True)
 axs[0,0].legend(fontsize=7);fig.suptitle(day+' ordinary-flight comparisons; identical force-based hidden-state initialization, no per-candidate fitting');fig.tight_layout();fig.savefig(R/f'brake-replay-{day}.png',dpi=150)
(R/'brake-replay-results.json').write_text(json.dumps(rows,indent=2))

