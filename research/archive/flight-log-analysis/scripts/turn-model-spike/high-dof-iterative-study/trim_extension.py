from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from linear_modes import perturb
rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 for climb in [0.,.5]:
  d=np.load(R/f'v3-trim-modal-{flight}-{climb}-0.0.npz');sim=Simulator(flight);y,r=sim.replay(d['state'],np.tile(d['command'],(7501,1)),rpm=float(d['rpm']));t=np.arange(len(y))*.04;rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(y[:,8],y[:,7])),.04));bank=Rot.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,0];row=dict(flight=flight,climb=climb,last50_turn=float(rate[-1250:].mean()),last50_turn_std=float(rate[-1250:].std()),last50_climb=float(-y[-1250:,9].mean()),max_abs_bank=float(abs(bank).max()));rows.append(row);np.savez_compressed(R/f'trim-extension-{flight}-{climb}.npz',state=y,rpm=r,t=t,rate=rate,command=d['command']);axs[i,0].plot(t,rate,label=f'{climb} m/s climb');axs[i,1].plot(t,bank,label=f'{climb} m/s climb');print(row,flush=True)
 axs[i,0].set(title=flight.replace('_',' ').title(),ylabel='Course rate (deg/s)',xlabel='Time after numerical trim (s)');axs[i,1].set(ylabel='Payload bank (deg)',xlabel='Time after numerical trim (s)');axs[i,0].legend()
fig.suptitle('Does a 75-second numerical trim remain steady for another five minutes?');fig.tight_layout();fig.savefig(R/'trim-extension.png',dpi=150);(R/'trim-extension-results.json').write_text(json.dumps(rows,indent=2))
