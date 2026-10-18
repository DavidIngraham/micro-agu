from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import linear_sum_assignment
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
sim=Simulator('trout_lake');day='2026-02-15';a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');mask=slice(125,700);early=[840,960,1020];late=sorted(int(p.stem.split('-')[-2]) for p in R.glob(f'atlas-{day}-*-combined_small.npz') if int(p.stem.split('-')[-2])>=1800)
def inputs(start):
 i=np.searchsorted(a['t'],start);sl=slice(i,i+751);Y=a['Y'][sl];wind=a['wind'][i];u=np.c_[a['U'][sl,0],a['brake_mm'][sl]];speed=np.linalg.norm(Y[:,6:8]-wind[:2],axis=1);features=np.r_[u[mask].mean(axis=0),prop['rpm'][sl][mask].mean(),speed[mask].mean(),-Y[mask,8].mean()];return i,sl,Y,wind,u,features
scale=np.array([.05,5,5,500,.5,.5]);cost=np.array([[np.linalg.norm((inputs(e)[-1]-inputs(l)[-1])/scale) for l in late] for e in early]);ii,jj=linear_sum_assignment(cost);pairs=[dict(early=early[i],late=late[j],distance=float(cost[i,j]),early_features=inputs(early[i])[-1].tolist(),late_features=inputs(late[j])[-1].tolist()) for i,j in zip(ii,jj)];(R/'tl-matched-input-selection.json').write_text(json.dumps(dict(method='Distinct late ordinary windows >=1800s selected by minimum total standardized input/operating-point distance; no prediction errors used. Features: throttle,leftmm,rightmm,RPM,estimated horizontal airspeed,climb. Scales .05,5,5,500,.5,.5. Match distance is descriptive, not proof of identical conditions.',pairs=pairs),indent=2))
def signals(Y,wind,force=None):
 phi,theta,psi=Y[:,:3].T;p,q,r=Y[:,3:6].T;v=Y[:,6:9];va=v-wind;course=np.unwrap(np.arctan2(v[:,1],v[:,0]));air=np.unwrap(np.arctan2(va[:,1],va[:,0]));headingrate=np.rad2deg((q*np.sin(phi)+r*np.cos(phi))/np.cos(theta));filt=lambda x:savgol_filter(x,101,3,axis=0);deriv=lambda x:savgol_filter(x,101,3,deriv=1,delta=.04,axis=0);normal=np.c_[-np.sin(air),np.cos(air),np.zeros(len(Y))];acc=deriv(v);lat=np.sum(acc*normal,axis=1);slip=np.rad2deg(np.arctan2(np.sin(psi-air),np.cos(psi-air)));return dict(heading=filt(headingrate),ground=np.rad2deg(deriv(course)),air=np.rad2deg(deriv(air)),bank=filt(np.rad2deg(phi)),slip=filt(slip),lat=lat,force_lat=None if force is None else filt(np.sum(force*normal,axis=1)))
rows=[]
for pair in pairs:
 fig,axs=plt.subplots(6,2,figsize=(14,16))
 for col,start in enumerate([pair['early'],pair['late']]):
  i,sl,Y,wind,u,feat=inputs(start);d=np.load(R/f'candidate_v3-{day}-{start}.npz');y=d['state'];rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(sim.geometry['imu'],(len(y),3))));modelY=np.c_[rot.as_euler('xyz'),rot.inv().apply(y[:,10:13]),v];obs=signals(Y,wind,a['force'][sl]);pred=signals(modelY,wind);t=np.arange(len(Y))*.04
  for key,style in [('heading','-'),('ground','--'),('air',':')]:axs[0,col].plot(t,obs[key],color='black',ls=style,label='Flight '+key)
  axs[0,col].plot(t,pred['air'],color='#197d9e',label='Model air');axs[0,col].plot(t,pred['heading'],color='#d88735',lw=.8,label='Model heading')
  for row,key in [(1,'bank'),(2,'slip'),(3,'lat')]:axs[row,col].plot(t,obs[key],color='black',label='Flight');axs[row,col].plot(t,pred[key],color='#197d9e',label='Model')
  axs[3,col].plot(t,obs['force_lat'],color='#d88735',alpha=.7,label='IMU horizontal force/mass')
  axs[4,col].plot(t,u[:,1],label='Left mm');axs[4,col].plot(t,u[:,2],label='Right mm');axs[5,col].plot(t,prop['rpm'][sl],label='Logged RPM');axs[5,col].plot(t,d['rpm'],label='Model RPM');other=axs[5,col].twinx();other.plot(t,u[:,0],color='gray',alpha=.5);other.set_ylabel('Throttle',color='gray')
  for row,label in enumerate(['Turn/heading rate (deg/s)','Payload bank (deg)','Heading minus air-course (deg)','Lateral acceleration (m/s²)','Physical brake travel (mm)','RPM']):axs[row,col].set(xlabel='Elapsed time (s)',ylabel=label);axs[row,col].grid(alpha=.2)
  axs[0,col].set_title(f"TL {start}s: {'early failure' if col==0 else 'input-matched late window'}");axs[0,col].legend(fontsize=7,ncol=2);axs[3,col].legend(fontsize=7);axs[4,col].legend(fontsize=7);axs[5,col].legend(fontsize=7)
  row=dict(start=start,pair_early=pair['early'],features=feat.tolist(),observed={k:float(obs[k][mask].mean()) for k in ['heading','ground','air','bank','slip','lat','force_lat']},model={k:float(pred[k][mask].mean()) for k in ['heading','ground','air','bank','slip','lat']});rows.append(row);print(row,flush=True)
 fig.suptitle('Existing-data diagnostic: shared inputs do not guarantee the same hidden canopy state');fig.tight_layout(rect=(0,0,1,.97));fig.savefig(R/f"tl-matched-{pair['early']}-{pair['late']}.png",dpi=130)
(R/'tl-matched-diagnostic.json').write_text(json.dumps(rows,indent=2))
