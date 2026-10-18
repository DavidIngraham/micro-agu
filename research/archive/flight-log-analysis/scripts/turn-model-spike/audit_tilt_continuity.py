from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
O=Path(__file__).parent;ns={'__file__':str(O/'fit_canopy_tilt.py')};exec((O/'fit_canopy_tilt.py').read_text().split('lo=np.r_')[0],ns);fit=json.loads((O/'canopy-tilt-selected.json').read_text());p=fit['parameters'];fast=ns['fast'];fig,axs=plt.subplots(2,4,figsize=(16,8),constrained_layout=True);report=[]
for b,s in enumerate(ns['blocks']):
 g=s['g'];params=p|g['parameters'];x=np.array(fit['node_adjustments'][b]);chunks=[];gaps=[];ar=axs.flat[b];t=np.arange(376)*.04;ar.plot(t,np.rad2deg(s['truth'][:,0]),'k',label='Logged')
 for j in range(5):
  start=ns['node_state'](s,j,x[j]);y,r=fast.causal_replay(start,s['U'][75*j:75*j+76],params,g['mass'],s['wind'],s['voltage'],s['rpms'][75*j],substeps=32);chunks.append(y);ar.plot(t[75*j:75*j+76],np.rad2deg(R.from_quat(y[:,3:7]).as_euler('xyz')[:,0]),color='C2',alpha=.8,label='Shooting segments' if j==0 else None)
  if j<4:
   nxt=ns['node_state'](s,j+1,x[j+1]);end=y[-1];gaps.append(dict(position_m=float(np.linalg.norm(end[:3]-nxt[:3])),payload_orientation_deg=float(np.rad2deg((R.from_quat(nxt[3:7]).inv()*R.from_quat(end[3:7])).magnitude())),velocity_m_s=float(np.linalg.norm(end[7:10]-nxt[7:10])),canopy_orientation_deg=float(np.rad2deg((R.from_quat(nxt[16:20]).inv()*R.from_quat(end[16:20])).magnitude())),canopy_rate_rad_s=float(np.linalg.norm(end[23:26]-nxt[23:26]))))
 row=dict(day=s['day'],start=s['start'],node_gaps=gaps)
 try:
  full,r=fast.causal_replay(ns['node_state'](s,0,x[0]),s['U'],params,g['mass'],s['wind'],s['voltage'],s['rpm'],substeps=32);rp=R.from_quat(full[:,3:7]);arm=rp.apply(np.broadcast_to(g['imu'],(376,3)));v=full[:,7:10]+np.cross(full[:,10:13],arm);pos=full[:,:3]+arm;gyro=rp.inv().apply(full[:,10:13]);row.update(finite=True,position_rmse_m=float(np.sqrt(np.mean(np.sum((pos-s['positions'])**2,axis=1)))),bank_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(np.angle(np.exp(1j*(rp.as_euler('xyz')[:,0]-s['truth'][:,0])))**2)))),velocity_component_rmse_m_s=float(np.sqrt(np.mean((v-s['truth'][:,6:9])**2))),gyro_component_rmse_deg_s=float(np.rad2deg(np.sqrt(np.mean((gyro-s['truth'][:,3:6])**2)))));ar.plot(t,np.rad2deg(rp.as_euler('xyz')[:,0]),color='C0',label='Uninterrupted replay');np.savez_compressed(O/f'tilt-training-{b}.npz',states=full,rpm=r,truth=s['truth'],t=t)
 except (ValueError,FloatingPointError):row['finite']=False;ar.text(.03,.94,'Uninterrupted replay diverged',transform=ar.transAxes,va='top',color='C0')
 ar.set(title=f"{'HR' if s['day'].endswith('03') else 'TL'} {s['start']:g}s",xlabel='Training time (s)',ylabel='Bank (deg)');ar.grid(alpha=.25);ar.legend(fontsize=7);report.append(row)
fig.suptitle('Training continuity audit: short fitted segments versus uninterrupted propagation');fig.savefig(O/'canopy-tilt-continuity.png',dpi=140);(O/'canopy-tilt-continuity.json').write_text(json.dumps(report,indent=2));ns['pool'].shutdown();print(json.dumps(report,indent=2))
