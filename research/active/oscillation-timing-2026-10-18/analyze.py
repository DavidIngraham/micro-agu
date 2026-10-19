import json,hashlib
from pathlib import Path
import numpy as np
from scipy.signal import welch,periodogram,spectrogram,coherence,butter,sosfiltfilt,find_peaks
from scipy.ndimage import uniform_filter1d
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
BASE=Path("/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09")
def read(path,kind):
 r=json.loads((path/(kind+".json")).read_text())
 if kind=="IMU":r=[x for x in r if x["I"]==0]
 return np.array([x["TimeUS"]/1e6 for x in r]),r
def interp(t,x,q):
 x=np.asarray(x)
 return np.column_stack([np.interp(q,t,x[:,k]) for k in range(x.shape[1])])
def summary(a):
 return dict(p10=float(np.quantile(a,.1)),median=float(np.median(a)),p90=float(np.quantile(a,.9)),min=float(np.min(a)),max=float(np.max(a)))
def main():
 results={};spectra={};all_windows=[];events=[]
 fig_fft,af=plt.subplots(1,2,figsize=(11,4),layout="constrained")
 for ax,(flight,folder) in zip(af,[("hood_river","2026-01-03 15-41-42"),("trout_lake","2026-02-15 15-38-50")]):
  path=BASE/folder
  tp,pos=read(path,"POS");alt=np.array([x["RelHomeAlt"] for x in pos])
  start=float(tp[alt>3][0]);end=float(tp[alt>3][-1])
  ti,imu=read(path,"IMU");tr,rc=read(path,"RCOU");tm,mode=read(path,"MODE");te,esc=read(path,"ESC")
  q=np.arange(start,end,.02);fs=50.
  mask=(ti>=start)&(ti<=end)
  assert np.max(np.diff(ti[mask]))<.06,"Do not interpolate across missing gyro segments"
  gyro=interp(ti,[[x[k] for k in ["GyrX","GyrY","GyrZ"]] for x in imu],q)*180/np.pi
  # Latest actual outputs, not future/interpolated commands.
  ri=np.maximum(0,np.searchsorted(tr,q,side="right")-1)
  rcvals=np.array([[x[k] for k in ["C1","C3","C4"]] for x in rc])[ri]
  throttle=(rcvals[:,0]-1000)/1000
  factor=np.pi*55/360*.1
  left=(rcvals[:,1]-800)*factor;right=(2400-rcvals[:,2])*factor
  common=(left+right)/2;diff=right-left
  u=np.column_stack([throttle,diff,common])
  modes=np.array([x["ModeNum"] for x in mode])[np.maximum(0,np.searchsorted(tm,q,side="right")-1)]
  f,psd=welch(gyro,fs=fs,nperseg=3200,noverlap=1600,axis=0)
  ff,fft=periodogram(gyro,fs=fs,window="hann",detrend="constant",axis=0)
  spectra[flight]=dict(welch_frequency=f.tolist(),welch_psd=psd.tolist(),fft_frequency=ff.tolist(),fft_psd=fft.tolist())
  ax.plot(ff,fft[:,2],color="#aab9c9",alpha=.6,lw=.5,label="Whole-flight FFT, yaw")
  for k,label,color in [(0,"Roll","#678e49"),(1,"Pitch","#dd8d30"),(2,"Yaw","#245a99")]:
   ax.plot(f,psd[:,k],label="Welch "+label,color=color)
  ax.set(xlim=(.3,8),yscale="log",ylim=(.01,1e5),title=flight.replace("_"," ").title(),xlabel="Frequency (Hz)",ylabel="Rate PSD ((deg/s)²/Hz)")
  ax.legend(fontsize=8)
  peaks=[]
  for k in range(3):
   ix,_=find_peaks(psd[:,k],distance=round(.15/(f[1]-f[0])))
   ix=ix[(f[ix]>=.5)&(f[ix]<=8)]
   ix=sorted(ix,key=lambda i:psd[i,k],reverse=True)[:5]
   peaks.append([dict(hz=float(f[i]),psd=float(psd[i,k])) for i in ix])
  filtered=sosfiltfilt(butter(3,[1.2,1.9],fs=50,btype="bandpass",output="sos"),gyro,axis=0)
  envelope=np.sqrt(np.maximum(uniform_filter1d(filtered**2,size=100,axis=0),0))
  change=np.zeros_like(u);change[50:]=abs(u[50:]-u[:-50])
  active=np.any(change>np.array([.08,10,5]),axis=1)
  windows=[]
  for i in range(250,len(q)-750,500):
   sl=slice(i,i+500)
   wf,wp=welch(gyro[sl],fs=50,nperseg=500,axis=0)
   b=(wf>=1.2-1e-9)&(wf<=1.9+1e-9);wide=(wf>=.8)&(wf<=5)
   rms=np.sqrt(np.sum(wp[b],axis=0)*(wf[1]-wf[0]))
   w=dict(flight=flight,start=float(q[i]),rms=rms.tolist(),peak_hz=float(wf[np.flatnonzero(wide)[np.argmax(wp[wide,2])]]),
     quiet=bool(not np.any(active[max(0,i-500):i+500])),manual=bool(np.all(modes[sl]==0)),command_range=np.ptp(u[sl],axis=0).tolist())
   windows.append(w)
  # Large output-change events, separated by at least 20 s.
  inds=np.where(active & ~np.r_[False,active[:-1]])[0];accepted=[];last=-100000
  for i in inds:
   if i<750 or i+750>=len(q) or i-last<1000:continue
   before=float(np.median(envelope[i-500:i-150,2]));after=float(np.median(envelope[i+150:i+500,2]))
   e=dict(flight=flight,time=float(q[i]),change=change[i].tolist(),before=before,after=after,ratio=after/max(before,.1),manual=bool(np.all(modes[i-500:i+500]==0)))
   accepted.append(e);last=i
  events+=accepted;all_windows+=windows
  rr=np.array([w["rms"][2] for w in windows]);quiet=[w["rms"][2] for w in windows if w["quiet"]]
  # Data-defined high-amplitude episode onsets; exclude short gaps via 10 s separation.
  threshold=float(np.quantile(envelope[250:-250,2],.9));high=envelope[:,2]>threshold
  onset=np.where(high & ~np.r_[False,high[:-1]])[0];burst=[];last=-100000
  for i in onset:
   if i<500 or i+250>=len(q) or i-last<500:continue
   burst.append(dict(time=float(q[i]),preceded_by_large_output_change=bool(np.any(active[i-500:i]))));last=i
  result=dict(start=start,end=end,duration=end-start,gyro_max_gap=float(np.max(np.diff(ti[mask]))),welch_peaks=peaks,windows=len(windows),yaw_band_rms=summary(rr),
    fraction_above_5=float(np.mean(rr>5)),fraction_above_10=float(np.mean(rr>10)),band_peak_fraction=float(np.mean([1.2-1e-9<=w["peak_hz"]<=1.9+1e-9 for w in windows])),
    quiet_windows=len(quiet),quiet_rms=summary(quiet) if quiet else None,
    event_count=len(accepted),event_ratio=summary([e["ratio"] for e in accepted]) if accepted else None,
    event_fraction_increase_50pct=float(np.mean([e["ratio"]>1.5 for e in accepted])) if accepted else None,
    high_envelope_threshold=threshold,burst_onsets=burst,
    hashes={k:hashlib.sha256((path/(k+".json")).read_bytes()).hexdigest() for k in ["IMU","RCOU","POS","MODE","ESC"]})
  # Nonoverlapping spectra localize the extra ~1.16 Hz energy without concatenating segments.
  lf,lt,lp=spectrogram(gyro[:,2],fs=50,nperseg=1000,noverlap=0,window="hann")
  low=(lf>=1.05-1e-9)&(lf<=1.25+1e-9)
  energy=lp[low].sum(0)*(lf[1]-lf[0])
  order=np.argsort(energy)[::-1]
  result["lower_band_top_windows"]=[dict(start=float(start+lt[i]-10),end=float(start+lt[i]+10),power=float(energy[i]),fraction=float(energy[i]/energy.sum())) for i in order[:10]]
  result["lower_band_top10percent_power_fraction"]=float(energy[order[:max(1,int(np.ceil(len(energy)*.1)))]] .sum()/energy.sum())
  med=np.median(lp,axis=1);ix=(lf>=.8)&(lf<=5)
  result["median_window_spectrum_peak_hz"]=float(lf[np.flatnonzero(ix)[np.argmax(med[ix])]])
  result["manual_windows"]=sum(w["manual"] for w in windows)
  result["manual_yaw_band_rms"]=summary([w["rms"][2] for w in windows if w["manual"]])
  cs=np.r_[0,np.cumsum(active)]
  result["preceding_activity_fraction_all_times"]=float(np.mean((cs[500:]-cs[:-500])>0))
  if flight=="trout_lake":
   sel=(q>=580)&(q<660)
   ef,ep=welch(gyro[sel],fs=50,nperseg=1000,axis=0)
   band=(ef>=.8)&(ef<=2)
   idx=np.flatnonzero(band)[np.argmax(ep[band,2])]
   couplings=[]
   for k,name in enumerate(["throttle","differential_brake","common_brake"]):
    cf,c=coherence(u[sel,k],gyro[sel,2],fs=50,nperseg=1000)
    couplings.append(dict(command=name,coherence=float(c[idx])))
   result["episode_580_660"]=dict(yaw_peak=float(ef[idx]),modes=np.unique(modes[sel]).tolist(),command_coherence=couplings)

  results[flight]=result
  fig,axes=plt.subplots(4,1,figsize=(13,9),layout="constrained",sharex=True)
  sf,st,sp=spectrogram(gyro[:,2],fs=50,nperseg=500,noverlap=250)
  axes[0].pcolormesh(start+st,sf,10*np.log10(sp+1e-8),shading="auto",cmap="magma",vmin=-10,vmax=35)
  axes[0].set(ylim=(.5,5),ylabel="Yaw frequency (Hz)",title=flight.replace("_"," ").title()+" — entire airborne interval")
  axes[1].plot(q,envelope[:,2],label="Yaw 1.2–1.9 Hz RMS");axes[1].plot(q,envelope[:,1],label="Pitch band RMS")
  axes[1].axhline(threshold,color="gray",ls=":",label="90th percentile yaw")
  axes[1].set(ylabel="Band RMS (deg/s)");axes[1].legend(fontsize=8)
  axes[2].plot(q,throttle*100,label="Throttle (%)",color="#bc6b21")
  axes[2].set(ylabel="Throttle (%)")
  axes[3].plot(q,diff,label="Differential brake");axes[3].plot(q,common,label="Common brake",alpha=.7)
  axes[3].set(ylabel="Commanded pull (mm)",xlabel="Flight log time (s)",xlim=(start,end));axes[3].legend(fontsize=8)
  for e in accepted:
   for axis in axes[1:]:axis.axvline(e["time"],color="gray",alpha=.12,lw=.6)
  fig.savefig(OUT/(flight+"-timeline.png"),dpi=160);plt.close(fig)
 fig_fft.savefig(OUT/"whole-flight-spectra.png",dpi=180);plt.close(fig_fft)
 for name,value in [("summary",results),("windows",all_windows),("events",events)]:
  (OUT/(name+".json")).write_text(json.dumps(value,indent=2))
 # Compress dense whole-flight FFT ordinates; retain all bins.
 import gzip
 with gzip.open(OUT/"spectra.json.gz","wt") as f:f.write(json.dumps(spectra,separators=(",",":")))
 print(json.dumps({f:{k:v for k,v in x.items() if k not in ["hashes","burst_onsets"]} for f,x in results.items()},indent=2))
if __name__=="__main__":main()
