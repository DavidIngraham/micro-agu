from pathlib import Path
import json,datetime
from zoneinfo import ZoneInfo
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;d=R/'2026-02-15 15-38-50';a=np.load(d/'aligned.npz');t=a['t']
windows=[('Damper 0.10; P=I=0',635,654),('Damper 0; P=I=0',659,681),('Damper 0; P=.02 I=.025',2505,2553),('Damper .05; logged filter 2 Hz',2595,2627),('Damper .02; logged filter 10 Hz',2655,2725)]
fig,ax=plt.subplots(3,2,figsize=(15,10),layout='constrained');results=[]
for label,start,end in windows:
 m=(t>=start)&(t<end);q=a['q'][m];u=a['thr'][m];n=min(len(q),512);f,p=signal.welch(q,25,nperseg=n)
 ax[0,0].plot(t[m]-start,q,label=label,lw=.75);ax[1,0].plot(t[m]-start,u,lw=.7);ax[0,1].semilogy(f,p,label=label)
 f2,p2=signal.welch(u,25,nperseg=n);ax[1,1].semilogy(f2,p2)
 bands={}
 for name,lo,hi in [('slow',.2,.55),('fast',.8,2)]:
  ff,pp=signal.welch(q,25,nperseg=min(len(q),512));mask=(ff>=lo)&(ff<=hi);bands[name+'_rms']=float(np.sqrt(np.trapz(pp[mask],ff[mask])))
 results.append(dict(label=label,start=start,end=end,q_rms=float(np.sqrt(np.mean(q*q))),throttle_sd=float(np.std(u)),**bands))
for j in [0,1]:ax[j,1].set_xlim(.1,2.5);ax[j,1].set_xlabel('Frequency (Hz)');ax[j,0].set_xlabel('Seconds from window start')
ax[0,0].set_ylabel('q (deg/s)');ax[1,0].set_ylabel('Throttle (%)');ax[0,1].set_ylabel('q PSD ((deg/s)²/Hz)');ax[1,1].set_ylabel('Throttle PSD (%²/Hz)');ax[0,1].legend(fontsize=8)
for name,col in [('TECS_PTCH_DAMP','C0'),('TECS_PG_THR_P','C1'),('TECS_PG_THR_I','C2')]:ax[2,0].step(t,a[name],label=name,color=col)
ax[2,0].set_xlim(400,780);ax[2,0].legend(fontsize=8);ax[2,0].set_xlabel('Boot time (s)')
ax[2,1].step(t,a['TECS_PTCH_DAMP'],label='Damper');ax[2,1].set_xlim(2480,2750);ax2=ax[2,1].twinx();ax2.step(t,a['TECS_PG_PR_FILT'],color='C3');ax2.set_ylabel('Logged cutoff (Hz)',color='C3');ax[2,1].set_xlabel('Boot time (s)');ax[2,1].set_ylabel('Damper gain')
for x in ax.flat:x.grid(alpha=.2)
fig.suptitle('February 15: recorded damper experiments (observational comparisons)');fig.savefig(R/'damper-comparison.png',dpi=140);plt.close(fig)
# Model diagnostic: archived local linear plant, explicit motor lag, no tuning to data.
model=json.loads(Path('/home/david/repos/ArduPilot-tinkering/scratch/paraglider-tests/pitch-joint/oracle-control/design.json').read_text());A6=np.array(model['A']);B6=np.array(model['B']).reshape(6);A=np.zeros((7,7));A[:6,:6]=A6;A[:6,6]=B6;A[6,6]=-1/.14;B=np.zeros((7,1));B[6,0]=1/.14;C=np.zeros((1,7));C[0,2]=180/np.pi
modes=[dict(real=float(v.real),imag=float(v.imag),freq_hz=float(abs(v.imag)/(2*np.pi)),damping=float(-v.real/abs(v)) if abs(v)>0 else None) for v in np.linalg.eigvals(A)]
z=np.load(R/'2026-01-03 15-41-42/aligned.npz');mask=(z['t']>=400)&(z['t']<600);tt=z['t'][mask];u=z['thr'][mask]/100;qq=z['q'][mask];dt=.04;tr=np.arange(len(tt))*dt
_,yp,_=signal.lsim((A,B,C,np.zeros((1,1))),u-np.mean(u),tr)
cut=tr>=20;err=float(np.sqrt(np.mean((yp[cut]-qq[cut])**2)));base=float(np.sqrt(np.mean(qq[cut]**2)));corr=float(np.corrcoef(yp[cut],qq[cut])[0,1])
fig,ax=plt.subplots(2,1,figsize=(13,7),layout='constrained');ax[0].plot(tt,qq,label='Recorded body q',lw=.8);ax[0].plot(tt,yp,label='Archived linear model; recorded throttle input',lw=.8);ax[0].set_ylabel('q (deg/s)');ax[0].legend();ax[0].set_xlim(500,560);ax[1].plot(tt,u*100);ax[1].set_ylabel('Throttle (%)');ax[1].set_xlabel('Boot time (s)');fig.suptitle('January 3 MANUAL segment: diagnostic replay, no parameter fitting')
for x in ax:x.grid(alpha=.2)
fig.savefig(R/'model-replay.png',dpi=140);plt.close(fig)
res=dict(windows=results,model_modes=modes,replay=dict(log='2026-01-03 15-41-42',start_s=400,end_s=600,motor_tau_s=.14,warmup_discard_s=20,rmse_deg_s=err,zero_prediction_rmse_deg_s=base,correlation=corr,warning='Uncalibrated throttle-to-thrust scale; zero initial perturbation; omitted disturbances and brake/turn forcing. Diagnostic only, not a fitted or validated plant.'))
(R/'model-comparison.json').write_text(json.dumps(res,indent=2));print(json.dumps(res,indent=2))


