"""ESC order/alias diagnostic; do not infer rotor phase from low-rate RPM."""
import os,json,hashlib
from pathlib import Path
import numpy as np
from scipy.signal import welch,spectrogram,coherence
from scipy.stats import spearmanr
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
BASE=Path(os.environ.get("MICROAGU_FLIGHT_ANALYSIS","/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09"))
FLIGHTS={"hood_river":("2026-01-03","2026-01-03 15-41-42"),"trout_lake":("2026-02-15","2026-02-15 15-38-50")}
ORDERS=[1,2,3,4,7,14,42]
def fold(f,fs):return np.abs((np.asarray(f)+fs/2)%fs-fs/2)
def read(p,kind):
    rows=json.loads((p/(kind+".json")).read_text())
    if kind=="IMU":rows=[r for r in rows if r["I"]==0]
    if kind=="ESC":rows=[r for r in rows if r["Instance"]==0]
    if kind=="VIBE":rows=[r for r in rows if r["IMU"]==0]
    t=np.array([r["TimeUS"]/1e6 for r in rows])
    return t,rows
def arr(rows,keys):return np.array([[r[k] for k in keys] for r in rows])
def interp(t,x,q):return np.column_stack([np.interp(q,t,x[:,k]) for k in range(x.shape[1])])
def correlation(x,y):
    if np.std(x)==0 or np.std(y)==0:return None
    return float(spearmanr(x,y).statistic)
def main():
    audit={};result=[];spectra={};example=[]
    for flight,(day,folder) in FLIGHTS.items():
        path=BASE/folder
        ti,imu=read(path,"IMU");te,esc=read(path,"ESC");tv,vibe=read(path,"VIBE")
        g=arr(imu,["GyrX","GyrY","GyrZ"])*180/np.pi
        a=arr(imu,["AccX","AccY","AccZ"])
        er=arr(esc,["RPM","RawRPM"])
        vb=arr(vibe,["VibeX","VibeY","VibeZ","Clip"])
        source=np.load(BASE/"turn-model-spike"/(day+"-causal-input.npz"))
        pars={r["Name"]:r["Value"] for r in json.loads((path/"PARM.json").read_text())}
        audit[flight]=dict(esc_messages=len(te),imu_messages=len(ti),esc_median_dt=float(np.median(np.diff(te))),
            imu_median_dt=float(np.median(np.diff(ti))),imu_instances=sorted(set(r["I"] for r in json.loads((path/"IMU.json").read_text()))),
            parameters={k:v for k,v in pars.items() if any(z in k for z in ["INS_GYRO_RATE","INS_GYRO_FILTER","INS_HNTCH_ENABLE","INS_HNTC2_ENABLE","INS_LOG_BAT_MASK","SERVO_BLH_POLES","SCHED_LOOP_RATE"])},
            reported_gyro_rate_hz_median=float(np.median([r["GHz"] for r in imu])),
            clip_total_change=float(vb[-1,3]-vb[0,3]),
            hashes={k:hashlib.sha256((path/(k+".json")).read_bytes()).hexdigest() for k in ["IMU","ESC","VIBE","PARM"]})
        fs=50.
        flight_rows=[]
        for start in np.arange(np.ceil(max(ti[0],te[0])/20)*20,min(ti[-1],te[-1])-20,20):
            q=np.arange(start,start+20,1/fs)
            valid=np.interp(q,source["t"],source["valid"].astype(float),left=0,right=0)
            mask=(te>=start)&(te<start+20);im=(ti>=start)&(ti<start+20)
            if valid.mean()<.95 or mask.sum()<100 or np.max(np.diff(te[mask]))>.3 or im.sum()<900 or np.max(np.diff(ti[im]))>.06:continue
            rpm=er[mask,0];raw=er[mask,1]
            if np.median(rpm)<1500:continue
            gg=interp(ti,g,q);aa=interp(ti,a,q)
            f,pg=welch(gg,fs=fs,nperseg=500,noverlap=250,axis=0)
            _,pa=welch(aa,fs=fs,nperseg=500,noverlap=250,axis=0)
            band=(f>=1.2-1e-9)&(f<=1.9+1e-9);broad=(f>=.8)&(f<=5)
            peak=float(f[np.flatnonzero(broad)[np.argmax(pg[broad,2])]])
            row=dict(flight=flight,start=float(start),rpm_median=float(np.median(rpm)),rpm_p10=float(np.quantile(rpm,.1)),rpm_p90=float(np.quantile(rpm,.9)),
                raw_rpm_median=float(np.median(raw)),effective_log_rate=float((im.sum()-1)/(ti[im][-1]-ti[im][0])),shaft_hz=float(np.median(rpm)/60),
                yaw_peak_hz=peak,band_rms=np.sqrt(pg[band].sum(0)*(f[1]-f[0])).tolist(),
                accel_band_rms=np.sqrt(pa[band].sum(0)*(f[1]-f[0])).tolist(),
                vibe_mean=interp(tv,vb[:,:3],q).mean(0).tolist(),
                clip_change=float(np.interp(q[-1],tv,vb[:,3])-np.interp(q[0],tv,vb[:,3])),
                rpm_aliases={str(n):dict(median=float(np.median(fold(n*rpm/60,fs))),band_occupancy=float(np.mean((fold(n*rpm/60,fs)>=1.2)&(fold(n*rpm/60,fs)<=1.9))),
                    peak_coincidence=float(np.mean(abs(fold(n*rpm/60,fs)-peak)<=.2)),
                    raw_peak_coincidence=float(np.mean(abs(fold(n*raw/60,fs)-peak)<=.2))) for n in ORDERS})
            # Low-frequency RPM modulation coherence is association, not shaft-order coherence.
            rr=np.interp(q,te,er[:,0])
            fc,co=coherence(rr,gg[:,2],fs=50,nperseg=250,noverlap=125)
            row["rpm_yaw_coherence_at_peak"]=float(np.interp(peak,fc,co))
            flight_rows.append(row)
            spectra[f"{flight}-{int(start)}"]=dict(frequency=f.tolist(),gyro_psd=pg.tolist(),accel_psd=pa.tolist())
        result+=flight_rows
        med=np.array([r["rpm_median"] for r in flight_rows]);pk=np.array([r["yaw_peak_hz"] for r in flight_rows]);br=np.array([r["band_rms"][2] for r in flight_rows])
        audit[flight]["windows"]=len(flight_rows)
        audit[flight]["rpm_range"]=[float(med.min()),float(med.max())]
        audit[flight]["peak_range"]=[float(pk.min()),float(pk.max())]
        audit[flight]["rpm_peak_spearman"]=correlation(med,pk)
        audit[flight]["rpm_band_amplitude_spearman"]=correlation(med,br)
        audit[flight]["vibe_band_amplitude_spearman"]=correlation(np.linalg.norm([r["vibe_mean"] for r in flight_rows],axis=1),br)
        audit[flight]["clip_windows"]=sum(r["clip_change"]>0 for r in flight_rows)
        # Median-RPM alias coincidence versus circular shifts; descriptive null, not independent p-values.
        null=[]
        for n in ORDERS:
            alias=fold(n*med/60,50)
            actual=float(np.mean(abs(alias-pk)<=.2))
            shifted=[float(np.mean(abs(np.roll(alias,s)-pk)<=.2)) for s in range(1,len(alias))]
            null.append(dict(order=n,actual=actual,shift_median=float(np.median(shifted)),shift_p95=float(np.quantile(shifted,.95)),
                within_window_coincidence=float(np.mean([r["rpm_aliases"][str(n)]["peak_coincidence"] for r in flight_rows]))))
        audit[flight]["alias_null"]=null
        sensitivity=[]
        for sample_rate in [49.5,50.,50.5]:
            for n in [1,2]:
                sensitivity.append(dict(sample_rate=sample_rate,order=n,coincidence=float(np.mean(abs(fold(n*med/60,sample_rate)-pk)<=.2))))
        audit[flight]["alias_sample_rate_sensitivity"]=sensitivity
        # A 90 s representative window: preserve continuous spectrum and overlay folded orders.
        start=438 if flight=="hood_river" else 1440
        q=np.arange(start,start+90,.02);gg=interp(ti,g,q)
        ff,tt,ps=spectrogram(gg[:,2],fs=50,nperseg=500,noverlap=450)
        fig,axes=plt.subplots(3,1,figsize=(11,8),layout="constrained",sharex=True)
        axes[0].plot(te,er[:,0],label="ESC RPM");axes[0].plot(te,er[:,1],alpha=.5,label="RawRPM")
        axes[0].set(ylabel="Mechanical RPM",title=flight.replace("_"," ").title());axes[0].legend()
        for ax,lim in zip(axes[1:],[(0,25),(0,5)]):
            ax.pcolormesh(start+tt,ff,10*np.log10(ps+1e-8),shading="auto",cmap="magma",vmin=-20,vmax=30,rasterized=True)
            for n,c in [(1,"cyan"),(2,"lime")]:
                # Scatter avoids drawing misleading vertical lines at folding discontinuities.
                ax.scatter(te,fold(n*er[:,0]/60,50),s=3,color=c,alpha=.65,label=f"{n}x shaft alias at 50 Hz")
            ax.set(ylim=lim,ylabel="Frequency (Hz)");ax.legend(fontsize=8,loc="upper right")
        axes[-1].set(xlim=(start,start+90),xlabel="Flight log time (s)")
        fig.savefig(OUT/f"{flight}-spectrogram.png",dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
    for ax,flight in zip(axes,FLIGHTS):
        rows=[r for r in result if r["flight"]==flight]
        ax.scatter([r["rpm_median"] for r in rows],[r["yaw_peak_hz"] for r in rows],c=[r["band_rms"][2] for r in rows],cmap="viridis",s=25)
        ax.axhspan(1.2,1.9,color="gray",alpha=.15)
        ax.set(title=flight.replace("_"," ").title(),xlabel="Median ESC RPM",ylabel="Strongest yaw peak, 0.8–5 Hz",ylim=(.7,5.1))
    fig.savefig(OUT/"rpm-peak.png",dpi=180);plt.close(fig)
    (OUT/"audit.json").write_text(json.dumps(audit,indent=2))
    (OUT/"windows.json").write_text(json.dumps(result,indent=2))
    (OUT/"spectra.json").write_text(json.dumps(spectra,separators=(",",":")))
    print(json.dumps(audit,indent=2))
if __name__=="__main__":main()
