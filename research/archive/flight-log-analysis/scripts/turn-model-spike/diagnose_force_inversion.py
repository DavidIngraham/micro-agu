from pathlib import Path
import numpy as np,json
from scipy.ndimage import gaussian_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
params=json.loads((O/'effective-flight-fit.json').read_text())['parameters'];summary=[]
fig,axes=plt.subplots(2,3,figsize=(15,9))
for row,day in enumerate(['2026-01-03','2026-02-15']):
 a=np.load(O/(day+'-physical-input.npz'));meta=np.load(O/(day+'.npz'));t=a['t'];y=a['Y'];m=1.385 if row==0 else 1.758
 wind=np.tile([1.04,.59,0.] if row==0 else [.1,.086,0.],(len(t),1))
 if row:wind[t>=2500]=[.217,-1.139,0.]
 v=gaussian_filter1d(y[:,6:9],5,axis=0)-wind;acc=np.gradient(gaussian_filter1d(y[:,6:9],5,axis=0),t,axis=0);V=np.linalg.norm(v,axis=1);ev=v/V[:,None]
 theta=y[:,1];psi=y[:,2];thrust_axis=np.column_stack([np.cos(theta)*np.cos(psi),np.cos(theta)*np.sin(psi),-np.sin(theta)])
 T=params['thrust_max']*np.clip(a['U'][:,0],0,1)**params['thrust_exponent']
 F=m*(acc-np.array([0,0,9.80665]))-T[:,None]*thrust_axis
 D=-np.sum(F*ev,axis=1);lift=F+D[:,None]*ev;L=np.linalg.norm(lift,axis=1);Q=.5*1.225*1.16*V**2;CL=L/Q;CD=D/Q
 gamma=np.arcsin(np.clip(-ev[:,2],-1,1));alpha=theta-gamma
 mask=meta['airborne'].astype(bool)&(V>2)&(np.abs(t-2500)>1)&(t>320 if row==0 else t>215)
 # Distinct independent-ish samples for a legible point cloud; smoothing spans 0.2 s.
 ix=np.where(mask)[0][::12];ax=axes[row,0];sc=ax.scatter(np.rad2deg(alpha[ix]),CL[ix],c=np.rad2deg(np.abs(y[ix,0])),s=8,alpha=.4,cmap='viridis',vmin=0,vmax=50);ax.set(xlabel='Payload pitch − flight-path angle (deg)',ylabel='Inferred lift coefficient',ylim=(0,4),xlim=(-25,50),title=('Hood River' if row==0 else 'Trout Lake'));ax.grid(alpha=.2)
 ax=axes[row,1];ax.scatter(V[ix],CL[ix],c=np.rad2deg(np.abs(y[ix,0])),s=8,alpha=.4,cmap='viridis',vmin=0,vmax=50);ax.set(xlabel='Estimated airspeed (m/s)',ylabel='Inferred lift coefficient',ylim=(0,4));ax.grid(alpha=.2)
 ax=axes[row,2];ax.scatter(CL[ix]**2,CD[ix],c=np.rad2deg(np.abs(y[ix,0])),s=8,alpha=.4,cmap='viridis',vmin=0,vmax=50);ax.set(xlabel='Inferred lift coefficient squared',ylabel='Inferred drag coefficient',xlim=(0,8),ylim=(-.4,1.2));ax.grid(alpha=.2)
 summary.append(dict(day=day,samples=len(ix),alpha_CL_correlation=float(np.corrcoef(alpha[ix],CL[ix])[0,1]),negative_drag_fraction=float(np.mean(D[ix]<0)),CL_median=float(np.median(CL[ix])),CD_median=float(np.median(CD[ix]))))
 np.savez_compressed(O/(day+'-force-inversion.npz'),t=t,alpha=alpha,CL=CL,CD=CD,V=V,mask=mask,T=T)
fig.suptitle('Force inversion: can payload pitch explain canopy lift? (color = absolute bank)',fontsize=16);fig.tight_layout(rect=(0,0,1,.95));fig.savefig(O/'iteration-force-inversion.png',dpi=150);plt.close(fig)
(O/'force-inversion-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
