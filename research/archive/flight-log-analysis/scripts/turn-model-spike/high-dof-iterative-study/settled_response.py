from pathlib import Path
import sys,json,numpy as np
from scipy.spatial.transform import Rotation as Rot
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;sys.path.insert(0,str(R/'candidate-v1'));from simulator import Simulator
fig,axes=plt.subplots(4,2,figsize=(13,12),sharex=True);rows=[]
for j,flight in enumerate(['hood_river','trout_lake']):
 sim=Simulator(flight,left_slack_mm=0,yaw='mass_linear');base,rr=sim.replay(sim.initial_state(),np.tile([.4,0,0],(3001,1)));t=np.arange(1501)*.04
 for name in ['reference','common_bank','relative_yaw','right_brake']:
  state=base[-1].copy();u=np.tile([.4,0,0],(len(t),1))
  if name=='common_bank':
   dq=Rot.from_rotvec(Rot.from_quat(state[3:7]).apply([np.deg2rad(1),0,0]));rp=dq*Rot.from_quat(state[3:7]);rc=dq*Rot.from_quat(state[16:20]);state[3:7]=rp.as_quat();state[16:20]=rc.as_quat();state[13:16]=state[:3]+rp.apply(sim.geometry['hinge'])-rc.apply([0,0,.85])
  if name=='relative_yaw':
   rc=Rot.from_quat(state[16:20])*Rot.from_euler('z',1,degrees=True);state[16:20]=rc.as_quat();state[13:16]=state[:3]+Rot.from_quat(state[3:7]).apply(sim.geometry['hinge'])-rc.apply([0,0,.85])
  if name=='right_brake':u[(t>=2)&(t<3),2]=1.
  y,r=sim.replay(state,u,rpm=rr[-1]);rp=Rot.from_quat(y[:,3:7]);rc=Rot.from_quat(y[:,16:20]);angles=rp.as_euler('xyz');body=rp.inv().apply(y[:,10:13]);heading=np.rad2deg((body[:,1]*np.sin(angles[:,0])+body[:,2]*np.cos(angles[:,0]))/np.cos(angles[:,1]));rel=np.rad2deg((rp.inv()*rc).as_rotvec());v=y[:,7:10];course=np.rad2deg(np.gradient(np.unwrap(np.arctan2(v[:,1],v[:,0])),.04));a=np.c_[np.rad2deg(angles[:,0]),heading,course,rel[:,2]]
  if name=='reference':reference=a.copy()
  diff=a-reference
  for i in range(4):axes[i,j].plot(t,diff[:,i],label=name)
  np.savez_compressed(R/f'settled-{flight}-{name}.npz',t=t,state=y,response=a,difference=diff)
  rows.append(dict(flight=flight,perturbation=name,max_abs=abs(diff).max(axis=0).tolist(),last10_rms=np.sqrt(np.mean(diff[-251:]**2,axis=0)).tolist()))
 for i,label in enumerate(['Payload bank change (deg)','Heading rate change (deg/s)','Course rate change (deg/s)','Relative yaw change (deg)']):axes[i,j].set_ylabel(label);axes[i,j].grid(alpha=.25);axes[i,j].set_xlim(0,15)
 axes[0,j].set_title(flight);axes[-1,j].set_xlabel('seconds after settled state')
axes[0,0].legend();fig.suptitle('Small disturbances around powered rotating trim; differences from unperturbed trajectory');fig.tight_layout();fig.savefig(R/'settled-response.png',dpi=150);(R/'settled-response.json').write_text(json.dumps(rows,indent=2))
