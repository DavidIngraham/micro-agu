from pathlib import Path
R=Path(__file__).parent
exec((R/'ordinary_replay.py').read_text().split('rows=[]')[0])
variants={'reference':{},'combined_small':dict(Clbeta=-.03,Cnbeta=.0075,Cnr=-.0125),'arc075':dict(canopy_half_arc=.75),'arc060':dict(canopy_half_arc=.6)}
rows=[];fig,axs=plt.subplots(2,3,figsize=(14,8))
for di,(day,start,cut) in enumerate([('2026-01-03',571,591.24385),('2026-02-15',3343,3363.333506)]):
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');g=bg(day,dict(name='nominal',ballast_below_original_payload_CG_m=.10,imu_above_original_payload_CG_m=.05));i=np.searchsorted(a['t'],start);sl=slice(i,i+1501);truth=a['Y'][sl];t=a['t'][sl]-start;mask=(t>=cut-start)&(t<=cut-start+10);wind=a['wind'][i];rpm=float(prop['rpm'][i]);voltage=float(prop['voltage'][i]);z=initialize(a,i,g,wind,rpm)
 u=np.column_stack([a['U'][sl,0],a['brake_mm'][sl]/1000]);u[:,1]=np.maximum(u[:,1]-p['left_offset'],0);u[:,2]=np.maximum(u[:,2]-p['right_offset'],0)
 for j,arr in enumerate([-truth[:,8],np.linalg.norm(truth[:,6:8],axis=1),np.rad2deg(truth[:,0])]):axs[di,j].plot(t,arr,'k',lw=2,label='flight')
 for name,change in variants.items():
  par=p|g['parameters']|dict(joint_k_yaw=.5,rotor_spin_inertia=0.,rotor_spin_sign=1.,canopy_mass=.19)|change
  try:y,r=fast.causal_replay(z,u,par,g['mass'],wind,voltage,rpm,substeps=32)
  except FloatingPointError:
   rows.append(dict(day=day,variant=name,status='diverged'));continue
  rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(g['imu'],(len(y),3))));bank=rot.as_euler('xyz',degrees=True)[:,0]
  rows.append(dict(day=day,variant=name,observed_glide_climb=float(np.mean(-truth[mask,8])),model_glide_climb=float(np.mean(-v[mask,2])),glide_climb_bias=float(np.mean(truth[mask,8]-v[mask,2])),whole_climb_rmse=float(np.sqrt(np.mean((truth[:,8]-v[:,2])**2)))))
  np.savez_compressed(R/f'glide-{day}-{name}.npz',state=y,rpm=r,truth=truth,controls=u,t=t,wind=wind,glide_mask=mask)
  for j,arr in enumerate([-v[:,2],np.linalg.norm(v[:,:2],axis=1),bank]):axs[di,j].plot(t,arr,label=name)
 for j,label in enumerate(['Climb (m/s)','Groundspeed (m/s)','Payload bank (deg)']):axs[di,j].axvspan(cut-start,cut-start+10,color='gray',alpha=.15);axs[di,j].set(title=day+' '+label,xlabel='Time (s)');axs[di,j].grid(alpha=.2)
axs[0,0].legend(fontsize=8);fig.suptitle('Continuous 60-second glide contexts; shaded interval is the historical glide anchor');fig.tight_layout();fig.savefig(R/'glide-contexts.png',dpi=160);(R/'glide-context-results.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))

