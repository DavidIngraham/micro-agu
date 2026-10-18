"""Reproduce TECS loop diagrams and portable equation figures."""
from pathlib import Path
import re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch
root=Path(__file__).resolve().parents[1]
out=root/"docs/assets/control-loops"
ink="#213747"
def canvas(title):
 global fig,ax
 fig,ax=plt.subplots(figsize=(18,8))
 ax.set(xlim=(0,18),ylim=(0,8));ax.axis("off")
 ax.text(9,7.5,title,ha="center",fontsize=20,weight="bold",color=ink)
def txt(x,y,s,size=12):ax.text(x,y,s,ha="center",va="center",fontsize=size,color=ink)
def box(x,y,w,h,s):
 ax.add_patch(Rectangle((x-w/2,y-h/2),w,h,fc="#eff4f7",ec=ink,lw=1.4));txt(x,y,s)
def wire(points):
 for a,b in zip(points[:-2],points[1:-1]):ax.plot([a[0],b[0]],[a[1],b[1]],color=ink,lw=1.4)
 ax.add_patch(FancyArrowPatch(points[-2],points[-1],arrowstyle="-|>",mutation_scale=14,color=ink,lw=1.4))
def summ(x,y):
 ax.add_patch(Circle((x,y),.22,fc="#fff3ce",ec=ink));txt(x,y,"Σ")
def save(name):
 fig.savefig(out/(name+".png"),dpi=160,bbox_inches="tight",facecolor="white")
 fig.savefig(out/(name+".svg"),bbox_inches="tight",facecolor="white")
 plt.close(fig)
canvas("Standard TECS: coordinate energy and energy balance")
box(1.8,5,2.8,1.2,"Height + airspeed\ndemands")
box(5.3,5,2.6,1.4,"Energy targets\nand errors")
wire([(3.2,5),(4,5)])
box(9,6,3.1,1,"Total-energy\ncontroller")
box(9,3.5,3.1,1,"Energy-balance\ncontroller")
wire([(6.6,5.3),(7,5.3),(7,6),(7.45,6)])
wire([(6.6,4.7),(7,4.7),(7,3.5),(7.45,3.5)])
box(12.6,6,2.4,1,"Throttle / motor")
box(12.6,3.5,2.4,1,"Pitch controller\n+ elevator")
wire([(10.55,6),(11.4,6)]);wire([(10.55,3.5),(11.4,3.5)])
txt(11,4.05,"Pitch demand",10)
box(16,4.7,2.4,2.3,"Airplane")
wire([(13.8,6),(14.4,6),(14.4,5.2),(14.8,5.2)])
wire([(13.8,3.5),(14.4,3.5),(14.4,4.2),(14.8,4.2)])
wire([(16,3.55),(16,1.6),(5.3,1.6),(5.3,4.3)])
txt(10.5,1.9,"Measured height, airspeed and their rates",12)
txt(9,.6,"Conceptual signal paths; energy weighting, limits and flight-stage logic omitted.",11)
save("tecs-standard")
canvas("Parafoil TECS: throttle-only altitude control")
box(1.4,5,2.2,1.2,"Altitude demand\nshaping")
box(4.1,5,2.2,1.2,"Height error / Tₕ\n+ trajectory rate\n+ climb/sink limits")
wire([(2.5,5),(3,5)])
txt(5.75,5.8,"$v_{z,c}$",12)
summ(6.2,5);wire([(5.2,5),(5.98,5)])
txt(5.94,5.28,"+");txt(6.48,4.65,"−")
box(8.2,5,2.2,.85,"$K_P + K_I / s$")
wire([(6.42,5),(7.1,5)])
box(8.2,6.5,2.2,.85,"Throttle feedforward")
wire([(5.5,5),(5.5,6.5),(7.1,6.5)])
summ(10.3,5);wire([(9.3,5),(10.08,5)])
wire([(9.3,6.5),(10.3,6.5),(10.3,5.22)])
txt(10.6,5.5,"+");txt(10,5.28,"+");txt(10.6,4.7,"−")
box(12.2,5,2.5,1,"Throttle limits\n+ motor")
wire([(10.52,5),(10.95,5)])
box(16,5,2.8,1.4,"Canopy + payload")
wire([(13.45,5),(14.6,5)])
box(12.2,3,2.5,.85,"$K_q F_q(s)$")
wire([(16,4.3),(16,3),(13.45,3)])
txt(14.9,3.35,"Payload pitch rate q",11)
wire([(10.95,3),(10.3,3),(10.3,4.78)])
wire([(17.4,5),(17.8,5),(17.8,1.9),(6.2,1.9),(6.2,4.78)])
txt(12.7,2.17,"Measured climb rate $v_z$",11)
wire([(17.4,5.35),(17.95,5.35),(17.95,1),(4.1,1),(4.1,4.4)])
txt(10,1.25,"Measured altitude h  →  negative height feedback",11)
txt(9,.35,"Simplified paths: P+feedforward slew limit, integrator bounds and damper clamp omitted. Pitch demand stays at trim.",10)
save("tecs-parafoil")
b=chr(92)
equations=[
 ("tecs-energy",r"E_T = gh + @frac{V^2}{2}","Specific energy: E_T = gh + V²/2."),
 ("tecs-height",r"v_{z,c} = @dot{h}_c + @frac{h_c-h}{T_h}","Requested climb rate = altitude trajectory rate + altitude error / height time constant."),
 ("tecs-throttle",r"u_T = u_{@mathrm{FF}}(v_{z,c}) + K_P e_v + K_I @int e_v@,dt - K_q F_q(s)q_{@mathrm{payload}}","Throttle = feedforward + proportional correction + integral correction − filtered pitch-rate damping."),
 ("tecs-damper",r"@Delta u_T(s) = -0.1@left(@frac{2@pi(1.5)}{s+2@pi(1.5)}@right)^2 q_{@mathrm{payload}}(s)","Pitch damping: gain −0.1 with two cascaded first-order 1.5 Hz low-pass filters.")]
for name,eq,alt in equations:
 fig=plt.figure(figsize=(14,1.3));fig.text(.5,.5,"$"+eq.replace("@",b)+"$",ha="center",va="center",fontsize=24,color=ink)
 fig.savefig(out/(name+".png"),dpi=180,bbox_inches="tight",facecolor="white");plt.close(fig)
