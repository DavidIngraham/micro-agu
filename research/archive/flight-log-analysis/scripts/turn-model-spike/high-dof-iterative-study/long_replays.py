from pathlib import Path
R=Path(__file__).parent
exec((R/'ordinary_replay.py').read_text().split('rows=[]')[0])
rows=[];fig,axs=plt.subplots(3,3,figsize=(14,11))
for si,(day,start) in enumerate([('2026-01-03',300),('2026-02-15',810),('2026-02-15',2670)]):
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05));i=np.searchsorted(a['t'],start);sl=slice(i,i+4501);truth=a['Y'][sl];t=np.arange(len(truth))*.04;wind=a['wind'][i];rpm=float(prop['rpm'][i]);voltage=float(prop['voltage'][i]);z=initialize(a,i,g,wind,rpm)
 u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]/1000]);u[:,1]=np.maximum(u[:,1]-p['left_offset'],0);u[:,2]=np.maximum(u[:,2]-p['right_offset'],0);ct=np.unwrap(np.arctan2(truth[:,7],truth[:,6]));ct-=ct[0]
 for j,arr in enumerate([np.rad2deg(ct),np.rad2deg(truth[:,0]),-truth[:,8]]):axs[si,j].plot(t,arr,'k',label='flight',lw=1.5)
 for name,step,change in [('reference',32,{}),('combined',32,dict(Clbeta=-.03,Cnbeta=.0075,Cnr=-.0125)),('combined64',64,dict(Clbeta=-.03,Cnbeta=.0075,Cnr=-.0125))]:
  par=p|g['parameters']|dict(joint_k_yaw=.5,rotor_spin_inertia=0.,rotor_spin_sign=1.,canopy_mass=.19)|change
  try:
   y,r=fast.causal_replay(z,u,par,g['mass'],wind,voltage,rpm,substeps=step);rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(g['imu'],(len(y),3))));c=np.unwrap(np.arctan2(v[:,1],v[:,0]));c-=c[0];bank=rot.as_euler('xyz',degrees=True)[:,0];ce=np.rad2deg(c-ct)
   row=dict(day=day,start=start,variant=name,status='finite',course_E_deg_s=float(2/180**2*np.trapz(abs(ce),t)),bank_rmse=float(np.sqrt(np.mean((bank-np.rad2deg(truth[:,0]))**2))),climb_rmse=float(np.sqrt(np.mean((v[:,2]-truth[:,8])**2))))
   if name=='combined':comparison=y.copy()
   if name=='combined64':row['max_state_difference_32_64']=float(np.max(abs(y-comparison)));row['max_velocity_difference_32_64']=float(np.max(abs(y[:,7:10]-comparison[:,7:10])))
   np.savez_compressed(R/f'long-{day}-{start}-{name}.npz',state=y,rpm=r,truth=truth,t=t,controls=u,wind=wind)
   for j,arr in enumerate([np.rad2deg(c),bank,-v[:,2]]):axs[si,j].plot(t,arr,label=name,lw=.8)
  except (FloatingPointError,ValueError) as e:row=dict(day=day,start=start,variant=name,status='diverged',error=str(e))
  rows.append(row);print(row,flush=True)
 for j,label in enumerate(['Relative course (deg)','Bank (deg)','Climb (m/s)']):axs[si,j].set(title=f'{day} {start}s: {label}',xlabel='Elapsed seconds');axs[si,j].grid(alpha=.2)
axs[0,0].legend(fontsize=7);fig.suptitle('Three-minute continuous replays; frozen initial wind/voltage, identical controls and initialization');fig.tight_layout();fig.savefig(R/'long-replays.png',dpi=150);(R/'long-replay-results.json').write_text(json.dumps(rows,indent=2))
