from pathlib import Path
import json,csv
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent
MODE={0:'MANUAL',1:'CIRCLE',5:'FBWA',7:'CRUISE',10:'AUTO',12:'LOITER',15:'GUIDED'}
def rows(d,k):
 p=d/(k+'.json');return json.loads(p.read_text()) if p.exists() else []
def arr(d,k,f,inst=None):
 a=rows(d,k)
 if inst is not None:a=[x for x in a if x.get('I',x.get('C',0))==inst]
 a=[x for x in a if f in x and 'TimeUS' in x]
 return np.array([x['TimeUS']/1e6 for x in a]),np.array([x[f] for x in a])
def interp(t,tx,x,step=False,maxgap=.3):
 if len(tx)==0:return np.full(len(t),np.nan)
 idx=np.argsort(tx);tx,x=tx[idx],x[idx];j=np.clip(np.searchsorted(tx,t,side='right')-1,0,len(tx)-1)
 y=x[j].astype(float) if step else np.interp(t,tx,x)
 y[(t<tx[0])|(t>tx[-1])|(t-tx[j]>maxgap)]=np.nan
 return y
def spans(mask):
 ix=np.flatnonzero(np.diff(np.r_[False,mask,False]));return list(zip(ix[::2],ix[1::2]))
def rms(x):return float(np.sqrt(np.mean(x*x)))
allseg=[];flights=[]
for inv in json.loads((R/'inventory.json').read_text()):
 d=R/Path(inv['file']).stem
 ti,qi=arr(d,'IMU','GyrY',0);t=np.arange(ti[0],ti[-1],.04)
 q=interp(t,ti,qi)*180/np.pi
 def get(k,f,step=False,gap=.3,inst=None):return interp(t,*arr(d,k,f,inst),step,gap)
 pitch=get('ATT','Pitch');roll=get('ATT','Roll');h=get('POS','RelHomeAlt');spd=get('GPS','Spd',gap=.5);fix=get('GPS','Status',True,.5)
 flying=get('STAT','isFlying',True,.5);armed=get('STAT','Armed',True,.5);mode=get('MODE','ModeNum',True,1e9)
 # MODE messages are sparse: hold final mode to end
 mr=rows(d,'MODE');mt=np.array([x['TimeUS']/1e6 for x in mr]);mv=np.array([x['ModeNum'] for x in mr]);mode=mv[np.clip(np.searchsorted(mt,t,side='right')-1,0,len(mt)-1)]
 thr=get('CTUN','ThO');climb=-get('XKF1','VD',inst=0);pwm=get('RCOU','C1');ax=get('IMU','AccX',inst=0);az=get('IMU','AccZ',inst=0)
 airborne=(flying==1)&(armed==1)&(fix>=3)&((h>3)|(spd>3))&np.isfinite(q+pitch+roll+thr+climb)
 hist=rows(d,'PARM');pkeys=['TECS_PTCH_DAMP','TECS_PG_THR_P','TECS_PG_THR_I','TECS_PG_PR_FILT','THR_MAX','TRIM_THROTTLE','TECS_TIME_CONST','TECS_VERT_ACC','TECS_INTEG_GAIN','TECS_HDEM_TCONST','THR_SLEWRATE','TECS_CLMB_MAX','TECS_SINK_MAX','TECS_PG_ENABLE']
 ps={}
 for k in pkeys:
  pr=[x for x in hist if x['Name']==k];pt=np.array([x['TimeUS']/1e6 for x in pr]);pv=np.array([x['Value'] for x in pr])
  ps[k]=pv[np.clip(np.searchsorted(pt,t,side='right')-1,0,len(pt)-1)] if len(pr) else np.full(len(t),np.nan)
 boundaries=np.zeros(len(t),bool);boundaries[0]=True
 for a in [mode,*ps.values()]:boundaries[1:]|=np.isfinite(a[1:])&np.isfinite(a[:-1])&(a[1:]!=a[:-1])
 cuts=sorted(set([0,len(t),*np.flatnonzero(boundaries),*[v for z in spans(airborne) for v in z]]))
 segs=[]
 for a,b in zip(cuts[:-1],cuts[1:]):
  if b-a<250 or not np.all(airborne[a:b]):continue
  # trim 2 seconds after any mode/parameter/airborne boundary
  a+=50
  qq=q[a:b];uu=thr[a:b];nseg=min(1024,len(qq));f,p=signal.welch(qq,25,nperseg=nseg);band=(f>=.12)&(f<=2);peak=float(f[band][np.argmax(p[band])])
  fc,coh=signal.coherence(uu,qq,25,nperseg=min(256,len(qq)//3));co=float(np.interp(peak,fc,coh))
  s=dict(log=d.name,start_s=round(t[a],2),end_s=round(t[b-1],2),duration_s=round((b-a)*.04,2),mode=MODE.get(int(mode[a]),str(mode[a])),q_rms_deg_s=rms(qq),pitch_median_deg=float(np.median(pitch[a:b])),pitch_p05_deg=float(np.percentile(pitch[a:b],5)),pitch_p95_deg=float(np.percentile(pitch[a:b],95)),throttle_mean_pct=float(np.mean(uu)),throttle_sd_pct=float(np.std(uu)),climb_rms_mps=rms(climb[a:b]),roll_rms_deg=rms(roll[a:b]),peak_hz=peak,coherence_at_peak=co,**{k:float(v[a]) if np.isfinite(v[a]) else None for k,v in ps.items()})
  segs.append(s)
 intervals=[dict(start_s=float(t[a]),end_s=float(t[b-1]),duration_s=(b-a)*.04) for a,b in spans(airborne) if b-a>=250]
 valid=airborne
 info=dict(log=d.name,airborne_selected_s=float(sum(valid)*.04),intervals=intervals,max_relative_alt_m=float(np.nanmax(h)),gps_max_mps=float(np.nanmax(spd)),imu_median_hz=float(1/np.median(np.diff(ti))),imu_gaps_over_100ms=int(np.sum(np.diff(ti)>.1)),q_rms_airborne_deg_s=rms(q[valid]) if sum(valid) else None)
 flights.append(info);allseg+=segs
 np.savez_compressed(d/'aligned.npz',t=t,q=q,pitch=pitch,roll=roll,h=h,spd=spd,mode=mode,thr=thr,climb=climb,pwm=pwm,ax=ax,az=az,airborne=airborne,**ps)
 fig,axs=plt.subplots(5,1,figsize=(14,10),sharex=True,layout='constrained')
 axs[0].plot(t,h,lw=.7);axs[0].set_ylabel('Relative height (m)')
 axs[1].plot(t,pitch,lw=.6,label='Pitch');axs[1].plot(t,roll,lw=.5,alpha=.5,label='Roll');axs[1].set_ylim(-60,60);axs[1].legend(loc='upper right');axs[1].set_ylabel('Attitude (deg)')
 axs[2].plot(t,q,lw=.6);axs[2].set_ylim(-100,100);axs[2].set_ylabel('Body q (deg/s)')
 axs[3].plot(t,thr,lw=.6);axs[3].set_ylabel('Throttle (%)')
 axs[4].step(t,mode,lw=.7);axs[4].set_yticks(list(MODE),list(MODE.values()));axs[4].set_xlabel('Autopilot boot time (s)')
 for ax in axs:
  ax.grid(alpha=.2)
  for a,b in spans(airborne):
   if b-a>=250:ax.axvspan(t[a],t[b-1],color='green',alpha=.08)
 fig.suptitle(d.name+' | shaded: selected airborne data');fig.savefig(d/'overview.png',dpi=140);plt.close(fig)
 print(json.dumps(info),flush=True)
(R/'flight_metrics.json').write_text(json.dumps(flights,indent=2));(R/'segments.json').write_text(json.dumps(allseg,indent=2))
with (R/'segments.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(allseg[0]));w.writeheader();w.writerows(allseg)

