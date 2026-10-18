from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
from equilibrium import perturb,difference
rows=[];fig,axs=plt.subplots(2,2,figsize=(12,8))
for i,flight in enumerate(['hood_river','trout_lake']):
 for j,climb in enumerate([0.,.5]):
  sim=Simulator(flight);d=np.load(R/f'relative-trim-{flight}-{climb}.npz');base=d['state'];rpm=float(d['rpm']);u=d['command'];dt=.002;eps=1e-5;n,_=sim.step(base,rpm,u,dt=dt);jac=np.zeros((15,15))
  for k in range(15):
   x=np.zeros(15);x[k]=eps;yp,_=sim.step(perturb(base,x,sim.geometry),rpm,u,dt=dt);ym,_=sim.step(perturb(base,-x,sim.geometry),rpm,u,dt=dt);jac[:,k]=(difference(n,yp)-difference(n,ym))/(2*eps)
  poles=np.log(np.linalg.eigvals(jac).astype(complex))/dt;row=dict(flight=flight,climb=climb,poles_per_s=np.c_[poles.real,poles.imag].tolist(),max_growth_per_s=float(poles.real.max()));rows.append(row);print(flight,climb,sorted(poles,key=lambda p:-p.real)[:4],flush=True)
  for step in [-.5,.5]:
   command=u.copy();command[2]+=step
   if command[2]<0:command[1]=sim.parameters['left_offset']*1000-command[2];command[2]=0
   y,r=sim.replay(base,np.tile(command,(5001,1)),rpm=rpm);t=np.arange(len(y))*.04;rate=np.rad2deg(np.gradient(np.unwrap(np.arctan2(y[:,8],y[:,7])),.04));axs[i,j].plot(t,rate,label=f'{step:+g} mm right step');np.savez_compressed(R/f'equilibrium-step-{flight}-{climb}-{step}.npz',t=t,state=y,rate=rate)
  axs[i,j].set(title=f'{flight}: climb {climb}m/s',xlabel='Time after brake step (s)',ylabel='Course rate (deg/s)');axs[i,j].grid(alpha=.2);axs[i,j].legend()
fig.suptitle('Small brake steps around exact equilibria: transient and long-term response');fig.tight_layout();fig.savefig(R/'equilibrium-brake-steps.png',dpi=150);(R/'equilibrium-poles.json').write_text(json.dumps(rows,indent=2))
