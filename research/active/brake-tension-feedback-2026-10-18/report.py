
"""Compact figures and grouped diagnostics; raw arrays are reproducible outputs."""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from run import OUT

results=json.loads((OUT/"results.json").read_text())
labels=["baseline","frozen_aero","tension25","tension100","tension300"]
names=["Existing","Routing frozen","25 N/m","100 N/m","300 N/m"]
colors=["#c08a20","#777777","#78a9ce","#2878ad","#a42b42"]
summary={}
for flight in ["hood_river","trout_lake"]:
    group={}
    bases={r["name"]:r for r in results if r.get("flight")==flight and r["variant"]=="baseline"}
    for label in labels:
        rows=[r for r in results if r.get("flight")==flight and r["variant"]==label and "error" not in r]
        group[label]={
            "mean_rate_rmse_ratio":np.mean([np.array(r["uncentered_rate_rmse"])/np.array(bases[r["name"]]["uncentered_rate_rmse"]) for r in rows],axis=0).tolist(),
            "mean_band_rms":np.mean([r["metrics"]["model_band_rms"] for r in rows],axis=0).tolist(),
            "mean_observed_band_rms":np.mean([r["metrics"]["observed_band_rms"] for r in rows],axis=0).tolist(),
            "max_moment_Nm":np.max([r["moment_peak_Nm"] for r in rows],axis=0).tolist(),
        }
    summary[flight]=group
(OUT/"summary.json").write_text(json.dumps(summary,indent=2)+chr(10))

fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True,constrained_layout=True)
for i,ax in enumerate(axes):
    for label,name,col in zip(labels,names,colors):
        d=np.load(OUT/"traces"/f"hood_river-438-{label}.npz")
        ax.plot(d["time"],d["rates"][:,i+1],label=name,color=col,lw=1.2)
    ax.plot(d["time"],d["observed"][:,i+1],label="Flight",color="#202020",lw=1)
    ax.set_ylabel(["Pitch rate","Yaw rate"][i]+" (deg/s)")
    ax.set_xlim(442,460);ax.grid(alpha=.2)
axes[0].legend(ncol=3,fontsize=9)
axes[1].set_xlabel("Log time (s)")
fig.suptitle("Passive brake-tension reaction does not reproduce the missing oscillation")
fig.savefig(OUT/"powered-turn.png",dpi=160)

fig,axes=plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
for ax,flight in zip(axes,summary):
    x=np.arange(2);width=.12
    for j,(label,name,col) in enumerate(zip(labels,names,colors)):
        ax.bar(x+(j-2)*width,np.array(summary[flight][label]["mean_band_rms"])[1:],width,label=name,color=col)
    obs=summary[flight]["baseline"]["mean_observed_band_rms"][1:]
    ax.scatter(x,obs,color="black",marker="_",s=900,label="Flight")
    ax.set_xticks(x,["Pitch","Yaw"]);ax.set_title(flight.replace("_"," ").title())
    ax.set_ylabel("Mean 1.2–1.9 Hz RMS (deg/s)");ax.grid(axis="y",alpha=.2)
axes[0].legend(fontsize=8)
fig.savefig(OUT/"band-comparison.png",dpi=160)

fig,axes=plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
for ax,flight in zip(axes,summary):
    for label,name,col in zip(labels,names,colors):
        path=OUT/"traces"/f"constant-{flight}-prior-0.1-{label}-5.npz"
        if not path.exists():continue
        d=np.load(path);time=np.arange(len(d["delta"]))*.04
        ax.plot(time,d["delta"][:,1],label=name,color=col,lw=1.2)
    ax.set_xlim(0,5);ax.set_xlabel("Time after 5° disturbance (s)")
    ax.set_ylabel("Relative pitch deviation (deg)");ax.set_title(flight.replace("_"," ").title());ax.grid(alpha=.2)
axes[0].legend(fontsize=8)
fig.suptitle("Fixed servo commands: independently trimmed nominal operating points")
fig.savefig(OUT/"decay.png",dpi=160)
