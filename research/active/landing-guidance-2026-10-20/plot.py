import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/"summary.json").read_text())
plt.rcParams.update({"font.size":11,"axes.spines.top":False,"axes.spines.right":False})
fig,axes=plt.subplots(1,3,figsize=(13,4.5),constrained_layout=True)
for i,variant in enumerate(["stock","feedforward","feedback"]):
    values=[next(r for r in rows if r["id"]==f"{c}-{variant}") for c in ["calm","head","cross"]]
    for ax,key in zip(axes,["spot_error","sink","ground_speed"]):
        ax.bar(np.arange(3)+(i-1)*.25,[r.get(key,np.nan) for r in values],.25,label=variant)
        ax.set_xticks(np.arange(3),["Calm","Headwind","Crosswind"])
        ax.grid(axis="y",alpha=.2)
for ax,label in zip(axes,["Spot error (m)","Incoming sink (m/s)","Ground speed (m/s)"]):
    ax.set_ylabel(label)
axes[0].legend(fontsize=9)
fig.suptitle("LQI approach + experimental flare: position and touchdown severity")
fig.savefig(HERE/"comparison.png",dpi=170)
plt.close(fig)
fig,axes=plt.subplots(3,1,figsize=(9,8),constrained_layout=True)
for variant in ["stock","feedforward","feedback"]:
    r=json.loads((HERE/"runs"/f"calm-{variant}.json").read_text())
    y=np.array([s["state"] for s in r["samples"]])
    t=np.array([s["time"] for s in r["samples"]]);t-=t[-1]
    mask=t>=-5
    for ax,value in zip(axes,[-y[:,2],y[:,9],np.mean([s["brakes"] for s in r["samples"]],axis=1)]):
        ax.plot(t[mask],value[mask],label=variant)
for ax,label in zip(axes,["True AGL (m)","Sink (m/s)","Mean applied brake (mm)"]):
    ax.set_ylabel(label);ax.grid(alpha=.2)
axes[0].legend()
axes[-1].set_xlabel("Seconds before near-ground measurement")
fig.suptitle("Final five seconds — calm air, unchanged LQI gains")
fig.savefig(HERE/"flare.png",dpi=170)
plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
for variant in ["feedforward","feedback"]:
    selected=[next(r for r in rows if r["id"]==f"{c}-{variant}")
              for c in ["bias-low","calm","bias-high"]]
    for ax,key in zip(axes,["sink","ground_speed"]):
        ax.plot([-.25,0,.25],[r.get(key,np.nan) for r in selected],"o-",label=variant)
        ax.set_xlabel("AGL bias (m)")
        ax.grid(alpha=.2)
axes[0].set_ylabel("Incoming sink (m/s)")
axes[1].set_ylabel("Ground speed (m/s)")
axes[0].legend()
fig.suptitle("Height uncertainty affects both approach and flare timing")
fig.savefig(HERE/"height-bias.png",dpi=170)
