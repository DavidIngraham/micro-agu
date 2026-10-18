from pathlib import Path
import json,hashlib,shutil,sys,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent.parent;R=Path(__file__).parent;sys.path.insert(0,str(O));import up_away_model as A;import up_away_model_B2 as B
plan=json.loads((R/'reviewer-agreed-plan.json').read_text());base=R/'baseline';base.mkdir(exist_ok=True)
files=['up_away_model.py','up_away_model_B2.py','up-away-A1-fit.json','up-away-B2-fit.json','up-away-frozen-episodes.json','up-away-requirements.json']
for name in files:shutil.copy2(O/name,base/name)
manifest={name:hashlib.sha256((O/name).read_bytes()).hexdigest() for name in files};(base/'manifest.json').write_text(json.dumps(manifest,indent=2))
windows=[('2026-01-03',345,'HR-turn-reversal'),('2026-01-03',765,'HR-brake-release'),('2026-02-15',545,'TL-strong-release'),('2026-02-15',2785,'TL-alternating-brakes'),('2026-01-03',571,'HR-glide-context'),('2026-02-15',3343,'TL-glide-context')];records=[]
for day,start,label in windows:
 a=np.load(O/f'{day}-causal-input.npz');prop=np.load(O/f'{day}-propulsion-9x45.npz');t=a['t'];i=np.searchsorted(t,start);n=min(1501,len(t)-i);ix=slice(i,i+n);y=a['Y'][ix];u=a['U'][ix];tt=t[ix]-t[i];w=a['wind'][i,:2];av=y[:,6:8]-w;hist=a['Y'][i-50:i+1,6:8]-w;ph=np.unwrap(np.arctan2(hist[:,1],hist[:,0]));tm=np.arange(51)*.04-2;co=np.polyfit(tm,ph,2);groundcourse=np.unwrap(np.arctan2(y[:,7],y[:,6]));aircourse=np.unwrap(np.arctan2(av[:,1],av[:,0]));init=[np.linalg.norm(av[0]),-y[0,8],aircourse[0],co[1]]
 D=dict(U=u[None],initial=np.array([init]),wind=w[None],mass=np.array([1.385 if day=='2026-01-03' else 1.758]),day=np.array([int(day!='2026-01-03')]),voltage=np.array([prop['voltage'][i]]),rpm=np.array([prop['rpm'][i]]),initial_qdot=np.array([2*co[0]]))
 outs={}
 for name,module in [('A1',A),('B2',B)]:
  p=np.array(list(json.loads((O/f'up-away-{name}-fit.json').read_text())['parameters'].values()));outs[name]=module.predict(p,D)[0]
 fig,ax=plt.subplots(6,1,figsize=(12,13),sharex=True)
 ax[0].plot(tt,u[:,1]*72,label='left mm');ax[0].plot(tt,u[:,2]*72,label='right mm');ax[0].set_ylabel('Brake mm');ax[0].legend(loc='upper right')
 ax[1].plot(tt,u[:,0],label='throttle');ax[1].set_ylabel('Throttle fraction');twin=ax[1].twinx();twin.plot(tt,prop['rpm'][ix],color='gray',alpha=.7,label='measured RPM');twin.set_ylabel('RPM');ax[1].legend(loc='upper left')
 ax[2].plot(tt,np.rad2deg(y[:,0]),label='payload bank');ax[2].plot(tt,np.rad2deg(y[:,5]),label='body yaw gyro');ax[2].set_ylabel('deg / deg/s');ax[2].legend(loc='upper right')
 ax[3].plot(tt,np.rad2deg(groundcourse-groundcourse[0]),label='ground course observed',color='k');ax[3].plot(tt,np.rad2deg(aircourse-aircourse[0]),label='air course inferred',alpha=.5)
 for name,z in outs.items():ax[3].plot(tt,np.rad2deg(z[:,2]),label=name)
 ax[3].set_ylabel('Course change deg');ax[3].legend(loc='upper right')
 ax[4].plot(tt,np.linalg.norm(y[:,6:8],axis=1),label='ground speed',color='k');ax[5].plot(tt,-y[:,8],label='observed climb',color='k')
 for name,z in outs.items():ax[4].plot(tt,z[:,0],label=name);ax[5].plot(tt,z[:,1],label=name)
 ax[4].set_ylabel('Speed m/s');ax[5].set_ylabel('Climb m/s');ax[4].legend(loc='upper right');ax[5].legend(loc='upper right');ax[5].set_xlabel('Seconds from window start')
 for aa in ax:aa.grid(alpha=.2)
 fig.suptitle(f'{label} | {day} boot {t[i]:.2f}s | continuous baseline replay, no resets\nHistorical wind assumption; diagnostic windows, not yet frozen ordinary/stress atlas')
 fig.tight_layout(rect=[0,0,1,.96]);fig.savefig(R/f'{label}.png',dpi=140);plt.close(fig)
 records.append(dict(day=day,start_s=float(t[i]),duration_s=float(tt[-1]),label=label,valid_fraction=float(a['valid'][ix].mean()),wind=w.tolist(),initialization='Causal2s quadratic air-course q/qdot; measured initial speed/climb/course; held historical wind and initial voltage',purpose='Reviewer-requested diagnostic context; not final scorecard selection'))
(R/'initial-diagnostic-windows.json').write_text(json.dumps(records,indent=2));print(json.dumps(records,indent=2))
