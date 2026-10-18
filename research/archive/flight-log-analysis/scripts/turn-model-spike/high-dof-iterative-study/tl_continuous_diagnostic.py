from pathlib import Path
import sys,json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as Rot
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));sys.path.insert(0,str(R/'candidate-v3'));from simulator import Simulator
ns={'__file__':str(R/'ordinary_replay.py')};exec((R/'ordinary_replay.py').read_text().split('rows=[]')[0],ns);initialize=ns['initialize']
a=np.load(O/'2026-02-15-causal-input.npz');prop=np.load(O/'2026-02-15-propulsion-9x45.npz');fig,axs=plt.subplots(4,2,figsize=(14,11));rows=[]
for col,start in enumerate([810,2970]):
 i=np.searchsorted(a['t'],start);sl=slice(i,i+6001);truth=a['Y'][sl];wind=a['wind'][i];u=np.c_[a['U'][sl,0],a['brake_mm'][sl]];sim=Simulator('trout_lake');z=initialize(a,i,sim.geometry,wind,float(prop['rpm'][i]));y,r,applied=sim.replay_actuated(z,u,wind=wind,rpm=float(prop['rpm'][i]),voltage=float(prop['voltage'][i]));t=np.arange(len(y))*.04;rot=Rot.from_quat(y[:,3:7]);v=y[:,7:10]+np.cross(y[:,10:13],rot.apply(np.broadcast_to(sim.geometry['imu'],(len(y),3))));model=np.c_[rot.as_euler('xyz'),rot.inv().apply(y[:,10:13]),v]
 for name,Y,color in [('Flight',truth,'black'),('Model',model,'#197d9e')]:
  phi,theta=Y[:,0],Y[:,1];hr=np.rad2deg((Y[:,4]*np.sin(phi)+Y[:,5]*np.cos(phi))/np.cos(theta));va=Y[:,6:9]-wind;air=np.unwrap(np.arctan2(va[:,1],va[:,0]));rate=np.rad2deg(savgol_filter(air,101,3,deriv=1,delta=.04));bank=savgol_filter(np.rad2deg(phi),101,3);heading=savgol_filter(hr,101,3);axs[0,col].plot(t,rate,color=color,label=name+' air-course');axs[0,col].plot(t,heading,color=color,ls=':',alpha=.6,label=name+' heading');axs[1,col].plot(t,bank,color=color,label=name);axs[2,col].plot(t,np.rad2deg(air-air[0]),color=color,label=name);rows.append(dict(start=start,name=name,mean_air_turn=float(rate[125:-125].mean()),mean_bank=float(bank[125:-125].mean())))
 axs[3,col].plot(t,u[:,1],label='Left');axs[3,col].plot(t,u[:,2],label='Right');np.savez_compressed(R/f'tl-continuous-{start}.npz',state=y,rpm=r,truth=truth,controls=u,t=t,wind=wind)
 for j,label in enumerate(['Turn rate (deg/s)','Payload bank (deg)','Relative air-course (deg)','Physical brake travel (mm)']):axs[j,col].set(title=f'TL log {start}–{start+240}s' if j==0 else None,xlabel='Elapsed time (s)',ylabel=label);axs[j,col].grid(alpha=.2)
 axs[0,col].legend(fontsize=7);axs[3,col].legend(fontsize=7)
fig.suptitle('Continuous four-minute replay: early bias persists without30-second restarts');fig.tight_layout();fig.savefig(R/'tl-continuous-diagnostic.png',dpi=150);(R/'tl-continuous-diagnostic.json').write_text(json.dumps(rows,indent=2));print(rows)
