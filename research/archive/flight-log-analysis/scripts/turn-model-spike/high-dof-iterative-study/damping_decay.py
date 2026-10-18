from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from linear_modes import perturb,linearize
rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 d=np.load(R/f'relative-trim-{flight}-0.0.npz')
 for name,cy,cp in [('nominal',.02,.015),('yaw_half',.01,.015),('both_quarter',.005,.00375)]:
  sim=Simulator(flight);sim.parameters.update(joint_c_yaw=cy,joint_c_pitch=cp);modes=linearize(sim,d['state'],float(d['rpm']),d['command']);x=np.zeros(15);x[1]=np.deg2rad(1);y,r=sim.replay(perturb(d['state'],x,sim.geometry),np.tile(d['command'],(751,1)),rpm=float(d['rpm']));g=np.rad2deg(Rot.from_quat(y[:,3:7]).inv().apply(y[:,10:13]));t=np.arange(len(y))*.04;rows.append(dict(flight=flight,name=name,modes=modes));np.savez_compressed(R/f'damping-decay-{flight}-{name}.npz',t=t,gyro=g,state=y)
  for j,axis in enumerate([1,2]):axs[i,j].plot(t,g[:,axis],label=name,lw=1)
 for j,label in enumerate(['Pitch','Yaw']):axs[i,j].set(xlim=(0,10),title=f'{flight}: {label}',xlabel='Time after 1° payload pitch displacement (s)',ylabel='Body rate (deg/s)');axs[i,j].legend();axs[i,j].grid(alpha=.2)
fig.suptitle('Damping sensitivity: free response at level-flight equilibrium');fig.tight_layout();fig.savefig(R/'damping-free-response.png',dpi=150);(R/'damping-free-response.json').write_text(json.dumps(rows,indent=2))
