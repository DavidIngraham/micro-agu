"""Compare archived native baseline with the final-design candidate."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument("root",type=Path);a=p.parse_args()
records={}
for condition in ["calm","gust"]:
 for name,path in [("baseline",a.root.parent/"gust-control"/("two-pole-native-"+condition)),("selected",a.root/("native-"+condition))]:
  d=json.loads((path/"gust-metrics.json").read_text())
  assert d["parameters"]["TECS_VERT_ACC"]==(7 if name=="baseline" else 3.5)
  assert d["parameters"]["TECS_PG_PR_FILT"]==1.5 and d["parameters"]["TECS_PG_PR_FILT2"]==1.5
  assert d["parameters"]["PG_TURN_FILT"]==4
  assert not d["backend"]["errors"] and d["backend"]["clipped_packets"]==0
  log=path/("run.log" if name=="selected" else "ArduPlane-ParagliderAutoMission.txt")
  assert "PASSED" in log.read_text()
  records[condition+"-"+name]=d
for condition in ["calm","gust"]:
 b=records[condition+"-baseline"]["parameters"];c=records[condition+"-selected"]["parameters"]
 diff={k:[b.get(k),v] for k,v in c.items() if b.get(k)!=v}
 assert set(diff)=={"TECS_VERT_ACC"},diff
fig,axes=plt.subplots(3,2,figsize=(12,10),layout="constrained")
for col,condition in enumerate(["calm","gust"]):
 for row,(key,label) in enumerate([("height_rms_m","Height RMS (m)"),("pitch_rate_rms_deg_s","Pitch-rate RMS (deg/s)"),("throttle_std_pct","Throttle std (percentage points)")]):
  for j,name in enumerate(["baseline","selected"]):
   values=[records[condition+"-"+name]["loiters"][seq][key] for seq in ["3","8"]]
   axes[row,col].bar([j*.35,1+j*.35],values,width=.3,label=name)
  axes[row,col].set_xticks([.175,1.175],["CW","CCW"]);axes[row,col].set_ylabel(label);axes[row,col].grid(axis="y",alpha=.25)
 axes[0,col].set_title(condition);axes[0,col].legend()
fig.suptitle("Native AUTO: doubled climb PI, unchanged pitch filters and steering")
fig.savefig(a.root/"native-comparison.png",dpi=160)
(a.root/"native-results.json").write_text(json.dumps(records,indent=2))
for condition in ["calm","gust"]:
 for name in ["baseline","selected"]:
  d=records[condition+"-"+name];print(condition,name,"height",[round(d["loiters"][s]["height_rms_m"],4) for s in ["3","8"]],"corner",round(d["max_corner_overshoot_m"],3))