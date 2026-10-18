from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from equilibrium import solve,perturb
cal={x['flight']:x for x in json.loads((R/'soft-trim-calibration.json').read_text())};rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 for j,climb in enumerate([0.,.5]):
  d=np.load(R/f'equilibrium-trim-{flight}-{climb}-0.0.npz')
  for variant,color in [('nominal','#197d9e'),('soft_retrim','#d88735')]:
   sim=Simulator(flight)
   if variant=='soft_retrim':sim.parameters['joint_k_roll']=1.;sim.parameters['right_offset']=cal[flight]['matched_right_offset_m']
   root=solve(sim,d['state'],d['command'],climb=climb);assert abs(root['residual']).max()<1e-6
   for kind,style in [('bank_kick','-'),('right_step','--'),('left_step',':')]:
    z=root['state'];u=root['command'].copy()
    if kind=='bank_kick':
     x=np.zeros(15);x[0]=x[3]=np.deg2rad(1.);z=perturb(z,x,sim.geometry)
    else:
     u[2]+=.5 if kind=='right_step' else -.5
     if u[2]<0:u[1]=sim.parameters['left_offset']*1000-u[2];u[2]=0
    y,r=sim.replay(z,np.tile(u,(7501,1)),rpm=root['rpm']);t=np.arange(len(y))*.04;rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(y[:,8],y[:,7])),.04));row=dict(flight=flight,climb=climb,variant=variant,disturbance=kind,final_rate=float(rate[-500:].mean()),max_abs_rate=float(abs(rate).max()),root_residual=float(abs(root['residual']).max()),trim_command=root['command'].tolist());rows.append(row);axs[i,j].plot(t,rate,color=color,ls=style,label=f'{variant} {kind}');print(row,flush=True)
  axs[i,j].set(title=f'{flight}: {climb:g} m/s climb',xlabel='Time (s)',ylabel='Air-course rate (deg/s)');axs[i,j].grid(alpha=.2)
axs[0,0].legend(fontsize=7,ncol=2);fig.suptitle('Stiffness with a model-derived level-trim correction: +1° bank and ±0.5 mm brake disturbances');fig.tight_layout();fig.savefig(R/'soft-retrim-response.png',dpi=150);(R/'soft-retrim-response.json').write_text(json.dumps(rows,indent=2))
