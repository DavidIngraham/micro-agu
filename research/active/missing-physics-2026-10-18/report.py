
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from run import OUT

fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True,constrained_layout=True)
for ax,index in zip(axes,[1,2]):
 for label,name,color in [("baseline","Existing model","#bd8419"),("prop-0.6","Transverse propeller force","#2779b2"),("drag-z--1","Drag center 50 mm above CG","#4c9866")]:
    d=np.load(OUT/"traces"/f"hood_river-438-{label}.npz")
    ax.plot(d["time"],d["rates"][:,index],label=name,color=color,lw=1.4)
 ax.plot(d["time"],d["observed"][:,index],label="Flight",color="#202020",lw=1)
 ax.set_ylabel(["Roll","Pitch","Yaw"][index]+" rate (deg/s)")
 ax.grid(alpha=.2);ax.set_xlim(442,460)
axes[0].legend(fontsize=9,ncol=2)
axes[-1].set_xlabel("Log time (s)")
fig.suptitle("Hood River: tested propeller and payload-drag loads leave the oscillation missing")
fig.savefig(OUT/"aero-comparison.png",dpi=160)

fig,axes=plt.subplots(1,2,figsize=(10,4),constrained_layout=True)
for ax,flight in zip(axes,["hood_river","trout_lake"]):
 for label,name,color in [("baseline","Existing","#bd8419"),("prop-0.6","Propeller force","#2779b2"),("drag-z--1","Offset drag","#4c9866")]:
    d=np.load(OUT/"traces"/f"constant-{flight}-{label}-5.npz")
    ax.plot(np.arange(len(d["delta"]))*.04,d["delta"][:,1],label=name,color=color)
 ax.set_xlim(0,5);ax.set_title(flight.replace("_"," ").title())
 ax.set_xlabel("Time after disturbance (s)");ax.set_ylabel("Relative pitch deviation (deg)")
 ax.grid(alpha=.2)
axes[0].legend()
fig.suptitle("Fixed commands after a 5-degree disturbance: valid trimmed cases decay")
fig.savefig(OUT/"decay.png",dpi=160)

fig,ax=plt.subplots(figsize=(10,4),constrained_layout=True)
for label,name,color in [("baseline","Existing","#bd8419"),("risers-500-2","Unqualified compliant-bridle model","#a83242")]:
 d=np.load(OUT/"traces"/f"hood_river-438-{label}.npz")
 ax.plot(d["time"],d["rates"][:,2],label=name,color=color,lw=1)
ax.plot(d["time"],d["observed"][:,2],label="Flight",color="#202020",lw=1)
ax.set_xlim(440,460);ax.set_xlabel("Log time (s)");ax.set_ylabel("Body yaw rate (deg/s)")
ax.set_title("Rejected suspension result: more motion is not a matched oscillation")
ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.savefig(OUT/"unqualified-suspension.png",dpi=160)
