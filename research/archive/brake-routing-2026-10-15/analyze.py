import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
P=Path(__file__).parent
f=22.5
cases={"Original: 132 mm risers":20.5,"185 mm risers, holes unchanged":47.,"185 mm risers, holes moved outward":20.5}
def length(theta,h,d):return np.sqrt(f*f+d*d+h*h+2*f*h*np.sin(theta))
rows=[]
for h in [0,20,40,60,80]:
 old=length(0,h,20.5);new=length(0,h,47.)
 rows.append(dict(ring_height_mm=h,lower_segment_increase_mm=float(new-old),old_mm_per_deg=float(f*h/old*np.pi/180),wide_mm_per_deg=float(f*h/new*np.pi/180),old_change_at_20deg_mm=float(length(np.deg2rad(20),h,20.5)-old),wide_change_at_20deg_mm=float(length(np.deg2rad(20),h,47)-new)))
# Upper segment: canopy endpoint remains fixed; new ring moves outward 26.5 mm.
upper=[]
for lateral in [100,250,500]:
 for height in [800,1400]:
  delta=np.hypot(lateral-92.5,height)-np.hypot(lateral-66,height)
  upper.append(dict(canopy_endpoint_halfspan_mm=lateral,vertical_gap_mm=height,upper_segment_change_mm=float(delta)))
(P/'results.json').write_text(json.dumps(dict(lower_segment_sensitivity=rows,upper_segment_sensitivity=upper,assumptions="Illustrative geometry, not measured ring height or canopy brake endpoint. Ring follows canopy; payload rotates around riser bolt axis. Guides at same height as bolt; fore-aft offset 22.5 mm. No friction, slack, elasticity or dynamic tension."),indent=2))
fig,ax=plt.subplots(1,2,figsize=(11,4),layout="constrained")
t=np.linspace(-30,30,181)
for label,d in list(cases.items())[:2]:ax[0].plot(t,length(np.deg2rad(t),60,d)-length(0,60,d),label=label)
ax[0].set(xlabel="Relative payload pitch (degrees; defined sign)",ylabel="Guide-to-ring length change (mm)",title="Illustration: ring 60 mm above bolt (assumed)")
ax[0].legend(fontsize=8);ax[0].grid(alpha=.3)
h=np.linspace(0,80,161)
ax[1].plot(h,length(0,h,47)-length(0,h,20.5));ax[1].set(xlabel="Assumed D-ring height above bolt (mm)",ylabel="Extra guide-to-ring length (mm)",title="Wider risers with unchanged guide holes");ax[1].grid(alpha=.3)
fig.suptitle("Brake routing geometry sensitivity — not a flight prediction")
fig.savefig(P/'routing-sensitivity.png',dpi=160)
print(json.dumps(rows,indent=2));print(json.dumps(upper,indent=2))

# Verify analytic derivative against central differences and zero-height invariance.
for h in [0,20,40,60,80]:
 for d in [20.5,47.]:
  numerical=(length(1e-6,h,d)-length(-1e-6,h,d))/2e-6
  assert np.isclose(numerical,f*h/length(0,h,d),atol=1e-6)
assert np.allclose(length(np.deg2rad(t),0,47),length(0,0,47))
print("Derivative and zero-height checks passed")
