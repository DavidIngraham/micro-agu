import json,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
OUT=Path(__file__).resolve().parent
BASE=Path("/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09")
def score(y,p):
 e=p-y
 return dict(n=len(y),bias=float(e.mean()),mae=float(abs(e).mean()),rmse=float(np.sqrt(np.mean(e**2))),median_abs_pct=float(np.median(abs(e)/y)*100),p90_abs_pct=float(np.quantile(abs(e)/y,.9)*100))
def target(u,v):return 743.5778222336925*v*np.maximum(u-.0005141016498671315,0)**.9086807307118114
def main():
 result={}
 fig,axes=plt.subplots(2,2,figsize=(11,7),layout="constrained")
 for row,(flight,day,folder) in enumerate([("hood_river","2026-01-03","2026-01-03 15-41-42"),("trout_lake","2026-02-15","2026-02-15 15-38-50")]):
  path=BASE/folder
  esc=json.loads((path/"ESC.json").read_text());rc=json.loads((path/"RCOU.json").read_text())
  te=np.array([x["TimeUS"]/1e6 for x in esc]);rpm=np.array([x["RPM"] for x in esc]);raw=np.array([x["RawRPM"] for x in esc]);v=np.array([x["Volt"] for x in esc])
  tr=np.array([x["TimeUS"]/1e6 for x in rc]);u=np.array([(x["C1"]-1000)/1000 for x in rc])
  # Exact first-order evolution between logged command/voltage events; no future command interpolation.
  events=np.union1d(te,tr);events=events[events>=te[0]]
  pred=[];r=rpm[0];last=events[0];last_target=target(u[max(0,np.searchsorted(tr,last,side="right")-1)],v[0])
  for t in events:
   r=last_target+(r-last_target)*np.exp(-(t-last)/.01756125311487475)
   pred.append(r)
   i=max(0,np.searchsorted(tr,t,side="right")-1);j=max(0,np.searchsorted(te,t,side="right")-1)
   last_target=target(u[i],v[j]);last=t
  pred=np.interp(te,events,pred)
  causal=np.load(BASE/"turn-model-spike"/(day+"-causal-input.npz"))
  valid=np.interp(te,causal["t"],causal["valid"].astype(float),left=0,right=0)>.99
  valid&=(rpm>1500)&(v>10)
  age=te-tr[np.maximum(0,np.searchsorted(tr,te,side="right")-1)]
  valid&=age<.1
  for t in te[1:][np.diff(te)>.3]:valid&=~((te>=t)&(te<t+1))
  at=np.maximum(0,np.searchsorted(tr,te,side="right")-1)
  static=target(u[at],v)
  # Stable target: target variation across prior/following 0.3 s below 3%.
  stable=np.array([np.ptp(static[(te>=t-.3)&(te<=t+.3)])/max(y,1500)<.03 for t,y in zip(te,rpm)])
  result[flight]=dict(all=score(rpm[valid],pred[valid]),stable=score(rpm[valid&stable],pred[valid&stable]),changing=score(rpm[valid&~stable],pred[valid&~stable]),
    raw=score(raw[valid&(raw>1500)],pred[valid&(raw>1500)]),slewed_common_mask=score(rpm[valid&(raw>1500)],pred[valid&(raw>1500)]),raw_changing=score(raw[valid&~stable&(raw>1500)],pred[valid&~stable&(raw>1500)]),static=score(rpm[valid],static[valid]),
    input_hashes={k:hashlib.sha256((path/(k+".json")).read_bytes()).hexdigest() for k in ["ESC","RCOU"]})
  # Timing sensitivity is diagnostic, not a refit or a claim of physical motor lag.
  result[flight]["time_shifts"]=[dict(seconds=float(s),**score(rpm[valid],np.interp(te[valid]+s,te,pred))) for s in [-.12,-.08,-.04,0,.04,.08,.12]]
  axes[row,0].hexbin(rpm[valid],pred[valid],gridsize=45,mincnt=1,bins="log",cmap="viridis")
  axes[row,0].plot([0,13000],[0,13000],"k--",lw=1);axes[row,0].set(title=flight.replace("_"," ").title(),xlabel="Measured ESC RPM",ylabel="Predicted RPM")
  start,end=(438,468) if row==0 else (1440,1470)
  axes[row,1].plot(te,rpm,label="ESC RPM");axes[row,1].plot(te,raw,label="RawRPM",alpha=.7);axes[row,1].plot(te,pred,label="Model",alpha=.8)
  axes[row,1].set(xlim=(start,end),ylim=(0,13000),xlabel="Time (s)",ylabel="RPM");axes[row,1].legend()
  np.savez_compressed(OUT/(flight+".npz"),time=te,measured=rpm,predicted=pred,valid=valid,stable=stable)
 fig.savefig(OUT/"rpm-fit.png",dpi=180)
 (OUT/"results.json").write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
if __name__=="__main__":main()
