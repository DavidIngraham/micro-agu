"""Render the build article's functional avionics overview (not a pinout)."""
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
out=Path(__file__).resolve().parents[1]/"docs/assets/micro-agu"
fig,ax=plt.subplots(figsize=(14,10))
ax.set(xlim=(0,14),ylim=(0,10)); ax.axis("off")
ink="#213747"; power="#be5727"; signal="#26749b"
def box(x,y,w,h,title,detail):
 ax.add_patch(FancyBboxPatch((x-w/2,y-h/2),w,h,boxstyle="round,pad=0.08",fc="#f1f5f8",ec=ink,lw=1.3))
 ax.text(x,y+.18,title,ha="center",va="center",fontsize=13,weight="bold",color=ink)
 ax.text(x,y-.22,detail,ha="center",va="center",fontsize=10,color=ink)
def wire(points,color,label=None,at=None,dashed=False):
 for a,b in zip(points[:-2],points[1:-1]):
  ax.plot([a[0],b[0]],[a[1],b[1]],color=color,lw=2,ls="--" if dashed else "-")
 ax.add_patch(FancyArrowPatch(points[-2],points[-1],arrowstyle="-|>",mutation_scale=14,lw=2,color=color,linestyle="--" if dashed else "-"))
 if label: ax.text(*at,label,ha="center",va="center",fontsize=10,color=color,bbox=dict(fc="white",ec="none",pad=2))
ax.text(7,9.55,"Micro-AGU avionics",ha="center",fontsize=22,weight="bold",color=ink)
ax.text(7,9.12,"Power distribution, navigation inputs and actuator commands",ha="center",fontsize=12,color=ink)
box(2,8,3,1,"6S Li-ion battery","Two packs in parallel · 6 Ah total")
ax.add_patch(FancyBboxPatch((5.05,3.55),3.9,5.35,boxstyle="round,pad=0.08",fc="#fafcfd",ec=ink,lw=1.5,zorder=0))
ax.text(7,8.68,"Matek H743-WLITE",ha="center",fontsize=13,weight="bold",color=ink)
box(7,8,3,1,"Integrated power","Distribution + supply regulation")
box(12,8,2.5,1,"50 A BLHeli ESC","Motor power switching")
box(12,5.9,2.5,1,"FFTY 3115 motor","900 KV · 9 × 4.5 pusher prop")
box(7,4.6,3.3,1.5,"Flight controller","ArduPlane · navigation and control\nIMU / barometer · microSD logging")
box(2,5.8,3,1,"M10Q-5883","GNSS + compass")
box(2,3.4,3,1,"RadioMaster RP1","MAVLink mode")
box(5,1.8,3,1,"Left DS339HV","Brake capstan / left brake line")
box(9,1.8,3,1,"Right DS339HV","Brake capstan / right brake line")
wire([(3.58,8),(5.42,8)],power)
wire([(8.58,8),(10.67,8)],power)
wire([(12,7.42),(12,6.48)],power,"Motor power",(12,6.96))
wire([(7,7.42),(7,5.43)],power,"Regulated power",(7,6.6))
wire([(5.42,7.8),(4.2,7.8),(4.2,2.9),(5,2.9),(5,2.38)],power)
wire([(8.58,7.8),(10,7.8),(10,2.9),(9,2.9),(9,2.38)],power)
wire([(3.58,5.8),(4.7,5.8),(4.7,4.95),(5.27,4.95)],signal,"Position / heading",(3.9,6.55))
wire([(3.58,3.4),(4.7,3.4),(4.7,4.2),(5.27,4.2)],signal)
ax.add_patch(FancyArrowPatch((3.9,3.4),(3.58,3.4),arrowstyle="-|>",mutation_scale=14,lw=2,color=signal))
ax.text(4.7,2.65,"Control + telemetry",ha="center",fontsize=10,color=signal)
wire([(8.73,4.8),(10.4,4.8),(10.4,7.65),(10.67,7.65)],signal,"Throttle",(10.85,4.45))
wire([(6.3,3.77),(6.3,2.38)],signal,"PWM",(6.35,3.05))
wire([(7.7,3.77),(7.7,2.38)],signal,"PWM",(7.65,3.05))
ax.text(7,.85,"Orange: power     Blue: control / navigation signals",ha="center",fontsize=12,color=ink)
ax.text(7,.42,"Functional overview: connectors, pin assignments, grounds and peripheral supply wiring omitted.",
 ha="center",fontsize=10,color=ink)
fig.savefig(out/"avionics.png",dpi=180,bbox_inches="tight",facecolor="white")
fig.savefig(out/"avionics.svg",bbox_inches="tight",facecolor="white")

p=out/"avionics.svg"
p.write_text("\n".join(line.rstrip() for line in p.read_text().splitlines())+"\n")
