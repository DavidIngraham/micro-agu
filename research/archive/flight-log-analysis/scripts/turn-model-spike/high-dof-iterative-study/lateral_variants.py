from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O))
from run_sustained_envelope import initial,summary,p,bg
import fast_rotor_torque as fast
variants={'reference':{},'roll_damping':dict(Clp=-.1),'directional':dict(Cnbeta=.03,Cnr=-.05),'sideslip_roll':dict(Clbeta=-.03)}
rows=[];fig,axs=plt.subplots(2,3,figsize=(14,8),sharex=True)
for di,day in enumerate(['2026-01-03','2026-02-15']):
 g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05))
 for name,change in variants.items():
  par=p|g['parameters']|dict(joint_k_yaw=.5,rotor_spin_inertia=0.,rotor_spin_sign=1.,canopy_mass=.19)|change
  u=np.tile([.4,0.,0.],(2501,1));u[1000:1250,2]=.01;u[1750:2000,1]=max(.02-p['left_offset'],0)
  y,rpm=fast.causal_replay(initial(g,4.,0.),u,par,g['mass'],np.zeros(3),22.2,0.,substeps=32)
  np.savez_compressed(R/f'lateral-{day}-{name}.npz',state=y,rpm=rpm,controls=u)
  v=y[:,7:10];q=np.rad2deg(np.gradient(np.unwrap(np.arctan2(v[:,1],v[:,0])),.04));bank=Rot.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,0];cb=Rot.from_quat(y[:,16:20]).as_euler('xyz',degrees=True)[:,0]
  for j,a in enumerate([bank,cb,q]):axs[di,j].plot(np.arange(len(y))*.04,a,label=name)
  rows.append(dict(day=day,variant=name,change=change,neutral=summary(y[:1001]),final=summary(y)))
  print(day,name,flush=True)
for di,day in enumerate(['HR mass','TL mass']):
 for j,name in enumerate(['payload bank (deg)','canopy bank (deg)','course rate (deg/s)']):
  axs[di,j].set(title=day+' '+name,xlabel='Time (s)');axs[di,j].axvspan(40,50,color='green',alpha=.08);axs[di,j].axvspan(70,80,color='orange',alpha=.08);axs[di,j].grid(alpha=.2)
axs[0,0].legend(fontsize=8);fig.suptitle('Residual aerodynamic stability variants: neutral, 10 mm right brake, release, 20 mm commanded left brake, release');fig.tight_layout();fig.savefig(R/'lateral-variants.png',dpi=160)
(R/'lateral-variants.json').write_text(json.dumps(rows,indent=2))
