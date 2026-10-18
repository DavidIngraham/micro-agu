from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O))
from run_sustained_envelope import initial,summary,p,bg
import fast_rotor_torque as fast
rows=[];fig,axes=plt.subplots(2,3,figsize=(14,8),sharex=True)
for di,day in enumerate(['2026-01-03','2026-02-15']):
 g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05))
 for kr in [0,.4,p['joint_k_roll']]:
  for perturb in [-1,1]:
   z=initial(g,4.,0.);rot=Rot.from_euler('x',perturb,degrees=True)
   z[3:7]=rot.as_quat();z[16:20]=rot.as_quat();z[13:16]=rot.apply(g['hinge']-np.array([0,0,.85]))
   par=p|g['parameters']|dict(joint_k_roll=kr,joint_k_yaw=.5,rotor_spin_inertia=0.,rotor_spin_sign=0.,canopy_mass=.19)
   y,rpm=fast.causal_replay(z,np.tile([.4,0,0],(1501,1)),par,g['mass'],np.zeros(3),22.2,0.,substeps=32)
   row=dict(day=day,kr=kr,initial_bank_deg=perturb,**summary(y));rows.append(row)
   np.savez_compressed(R/f'stability-{day}-{kr}-{perturb}.npz',state=y,rpm=rpm)
   t=np.arange(len(y))*.04;v=y[:,7:10];course=np.rad2deg(np.gradient(np.unwrap(np.arctan2(v[:,1],v[:,0])),.04));pb=Rot.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,0];cb=Rot.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)[:,0]
   for j,a in enumerate([pb,cb,course]):axes[di,j].plot(t,a,label=f'k={kr:.2g}, initial {perturb:+d} deg')
  print(day,kr,flush=True)
for di,day in enumerate(['Hood River mass','Trout Lake mass']):
 for j,name in enumerate(['Payload bank (deg)','Canopy bank (deg)','Course rate (deg/s)']):axes[di,j].set(title=day+' / '+name,xlabel='Time (s)');axes[di,j].grid(alpha=.25)
axes[0,0].legend(fontsize=7);fig.suptitle('Neutral roll stability: no propeller reaction torque, symmetric zero brakes, +/-1 degree initial bank');fig.tight_layout();fig.savefig(R/'neutral-stability.png',dpi=160)
(R/'neutral-stability.json').write_text(json.dumps(rows,indent=2))
