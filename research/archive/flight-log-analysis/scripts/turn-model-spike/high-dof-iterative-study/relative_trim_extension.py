from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from linear_modes import perturb
rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 for j,climb in enumerate([0.,.5]):
  d=np.load(R/f'relative-trim-{flight}-{climb}.npz');sim=Simulator(flight)
  for kick in [0.,1.,-1.]:
   x=np.zeros(15);x[0]=x[3]=np.deg2rad(kick);state=perturb(d['state'],x,sim.geometry);y,r=sim.replay(state,np.tile(d['command'],(10001,1)),rpm=float(d['rpm']));t=np.arange(len(y))*.04;rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(y[:,8],y[:,7])),.04));row=dict(flight=flight,climb=climb,kick=kick,final_turn=float(rate[-500:].mean()),final_turn_std=float(rate[-500:].std()),final_climb=float(-y[-500:,9].mean()),max_abs_turn=float(abs(rate).max()));rows.append(row);np.savez_compressed(R/f'relative-trim-extension-{flight}-{climb}-{kick}.npz',t=t,state=y,rate=rate);axs[i,j].plot(t,rate,label=f'{kick:+g}° bank kick');print(row,flush=True)
  axs[i,j].set(title=f'{flight}: climb {climb}m/s',xlabel='Time from equilibrium (s)',ylabel='Course rate (deg/s)');axs[i,j].legend();axs[i,j].grid(alpha=.2)
fig.suptitle('Direct equilibrium solution: 400-second continuations and small disturbances');fig.tight_layout();fig.savefig(R/'relative-trim-extension.png',dpi=150);(R/'relative-trim-extension-results.json').write_text(json.dumps(rows,indent=2))
