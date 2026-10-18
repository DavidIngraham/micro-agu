"""Bounded full-loop longitudinal design study; outputs stay under Mission Planner.
Small-signal loops include fixed lateral speed scaling, rotor state, PI, height
feedback, derivative FF smoothing and two pitch filters. Saturations are nonlinear.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.signal import lsim
from scipy.linalg import eigvals
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument("--plants",type=Path,required=True);p.add_argument("--output",type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
f=np.geomspace(.001,15,1800);s=2j*np.pi*f
configs=[dict(name="baseline",q=.1,hz=1.5,vm=1,kh=.1)]
configs += [dict(name=f"damper-{q}-{hz}",q=q,hz=hz,vm=1,kh=.1) for q,hz in [(0.075,1.5),(.125,1.5),(.15,1.5),(.1,1),(.1,2)]]
configs += [dict(name=f"climb-{m}",q=.1,hz=1.5,vm=m,kh=.1) for m in [2,3]]
configs += [dict(name=f"height-{h}",q=.1,hz=1.5,vm=1,kh=h) for h in [.15,.2]]
def coupled(file,scale):
 d=np.load(file);A,B,C=[d[k] for k in ("A","B","C")]
 n=len(A);ac=np.zeros((n+3,n+3));ac[:n,:n]=A;w=2*np.pi*4
 ac[n:n+2,:n]=w*C[3:5];ac[n:n+2,n:n+2]=-w*np.eye(2);ac[n+2,n]=-.002
 ac[:n,n]=-scale*.005*B[:,1];ac[:n,n+1]=-scale*.003*B[:,1];ac[:n,n+2]=B[:,1]
 return ac,np.r_[B[:,0],0,0,0],np.pad(C,((0,0),(0,3)))
def close(A,B,C,cfg,voltage,gff):
 ratio=22.2/voltage;limit=int(min(60*ratio,100))/100;kv=limit/35*cfg["vm"];ki=.3*kv;kh=cfg["kh"];tc=.5
 if gff==.15:gff=limit-.45
 B=B*ratio;n=len(A);ac=np.zeros((n+5,n+5));ac[:n,:n]=A
 # appended states: h, throttle integral, q LPF1, q LPF2, smoothed altitude reference
 u=np.zeros(n+5);u[:n]=-kv*C[0];u[n]=-(gff+kv)*kh;u[n+1]=1;u[n+3]=-cfg["q"];u[n+4]=-(gff+kv)/tc
 direct=(gff+kv)*(kh+1/tc)
 ac[:n]+=np.outer(B,u);bc=np.zeros(n+5);bc[:n]=B*direct
 ac[n,:n]=C[0];ac[n+1,:n]=-ki*C[0];ac[n+1,n]=-ki*kh;ac[n+1,n+4]=-ki/tc;bc[n+1]=ki*(kh+1/tc)
 w=2*np.pi*cfg["hz"];ac[n+2,:n]=w*C[1];ac[n+2,n+2]=-w;ac[n+3,n+2]=w;ac[n+3,n+3]=-w
 ac[n+4,n+4]=-1/tc;bc[n+4]=1/tc
 cc=np.zeros((4,n+5));cc[0,n]=1;cc[1,:n]=C[0];cc[2,:n]=C[1];cc[3]=u
 dd=np.array([0,0,0,direct])
 return ac,bc,cc,dd,(kv,ki,gff,ratio)
def crossings(x,y,target):
 inds=np.where((y[:-1]-target)*(y[1:]-target)<0)[0]
 return [float(np.interp(target,sorted([y[i],y[i+1]]),[x[i],x[i+1]] if y[i]<y[i+1] else [x[i+1],x[i]])) for i in inds]
rows=[];details={};nominal={}
for file in sorted(a.plants.glob("*/plant.npz")):
 case=file.parent.name;voltage=20.4 if case=="low_voltage" else 22.2;gff=.18 if "descent" in case else .15
 for scale in [.9,1.15,1.4]:
  A,B,C=coupled(file,scale)
  H=np.array([C@np.linalg.solve(z*np.eye(len(A))-A,B) for z in s])
  for cfg in configs:
   ac,bc,cc,dd,(kv,ki,ff,ratio)=close(A,B,C,cfg,voltage,gff)
   eig=np.linalg.eigvals(ac);stable=bool(eig.real.max()<0)
   F=(2*np.pi*cfg["hz"]/(s+2*np.pi*cfg["hz"]))**2
   L=ratio*(H[:,0]*(kv+ki/s+(ff+kv+ki/s)*cfg["kh"]/s)+cfg["q"]*F*H[:,1])
   sensitivity=1/(1+L);mag=20*np.log10(abs(L));phase=np.unwrap(np.angle(L))*180/np.pi
   cross=crossings(np.log(f),mag,0);pm=[float(180+np.interp(x,np.log(f),phase)) for x in cross]
   row=dict(case=case,scale=scale,**cfg,stable=stable,max_real=float(eig.real.max()),Ms=float(max(abs(sensitivity))),gain_cross_hz=np.exp(cross).tolist(),phase_margins_deg=pm)
   if stable:
    t=np.linspace(0,100,2001);ref=.1*np.clip((t-2)/2,0,1)
    _,y,_=lsim((ac,bc[:,None],cc,dd[:,None]),ref,t)
    row.update(iae=float(np.trapz(abs(y[:,0]-ref),t)),height_peak=float(max(y[:,0])),q_peak_deg_s=float(max(abs(y[:,2]))*180/np.pi),throttle_peak=float(max(abs(y[:,3]))))
   rows.append(row)
   if case=="level" and scale==1.15:
    nominal[cfg["name"]]=(t,y,ref,ac,bc,cc,dd,L)
    details[cfg["name"]]=dict(poles=[[float(z.real),float(z.imag)] for z in eig],**row)
summary=[]
for cfg in configs:
 rs=[r for r in rows if r["name"]==cfg["name"]]
 summary.append(dict(**cfg,unstable=sum(not r["stable"] for r in rs),worst_real=max(r["max_real"] for r in rs),worst_Ms=max(r["Ms"] for r in rs),nominal=details[cfg["name"]]))
(a.output/"screen.json").write_text(json.dumps(dict(candidates=summary,rows=rows),indent=2))
for r in summary:print(r["name"],"unstable",r["unstable"],"worst pole",round(r["worst_real"],4),"Ms",round(r["worst_Ms"],2),"IAE",round(r["nominal"].get("iae",999),3),"q",round(r["nominal"].get("q_peak_deg_s",999),3),flush=True)
fig,ax=plt.subplots(2,2,figsize=(14,9),layout="constrained")
for name in ["baseline","damper-0.125-1.5","climb-2","height-0.15"]:
 t,y,r,ac,bc,cc,dd,L=nominal[name]
 ax[0,0].plot(t,y[:,0],label=name);ax[0,1].plot(t,y[:,2]*180/np.pi,label=name)
 ax[1,0].semilogx(f,20*np.log10(abs(L)),label=name);ax[1,1].semilogx(f,abs(1/(1+L)),label=name)
ax[0,0].plot(t,r,"k--",label="Shaped demand")
for aa,title,xlab in zip(ax.flat,["Altitude (m)","Payload pitch rate (deg/s)","Total throttle return ratio (dB)","Sensitivity magnitude"],["Time (s)","Time (s)","Frequency (Hz)","Frequency (Hz)"]):aa.set(title=title,xlabel=xlab);aa.grid(alpha=.25)
ax[0,0].legend(fontsize=8);fig.suptitle("Full-loop candidate screen: 0.1 m altitude ramp; level trim, frozen lateral scaling 1.15")
fig.savefig(a.output/"candidate-screen.png",dpi=160)
# Detailed final-design views: candidate doubles climb PI; other loops unchanged.
chosen=next(c for c in configs if c["name"]=="climb-2")
A,B,C=coupled(a.plants/"level/plant.npz",1.15)
names=["payload roll","payload pitch","relative roll","relative pitch","relative yaw",
       "forward velocity","side velocity","vertical velocity","payload roll rate","payload pitch rate","payload yaw rate",
       "canopy roll rate","canopy pitch rate","canopy yaw rate","rotor RPM","heading-rate filter","roll-rate filter","turn integral"]
eig,V=np.linalg.eig(A);W=np.linalg.inv(V);part=abs(V*W.T);part/=part.sum(axis=0)
modes=[]
for j,z in enumerate(eig):
 if z.imag < -1e-7:continue
 ix=np.argsort(part[:,j])[-3:][::-1]
 modes.append(dict(pole=[z.real,z.imag],frequency_hz=abs(z.imag)/(2*np.pi),damping=float(-z.real/abs(z)),states=[names[k] for k in ix],participation=part[ix,j].tolist()))
zeros={}
for i,label in [(0,"climb"),(1,"payload_pitch_rate"),(2,"relative_pitch_rate")]:
 n=len(A);M=np.block([[A,B[:,None]],[C[i:i+1],np.zeros((1,1))]]);N=np.zeros_like(M);N[:n,:n]=np.eye(n);z=eigvals(M,N);z=z[np.isfinite(z)]
 zeros[label]=[[float(v.real),float(v.imag)] for v in z if abs(v)<1e5]
(a.output/"plant-modes-zeros.json").write_text(json.dumps(dict(modes=modes,zeros=zeros,note="Full-order numerical transmission zeros; near pole-zero cancellations retained. Active lateral loop, frozen speed scaling 1.15."),indent=2))
fig,ax=plt.subplots(2,2,figsize=(14,10),layout="constrained")
for i,label in [(0,"Climb rate / throttle"),(1,"Payload pitch rate / throttle")]:
 H=np.array([C[i]@np.linalg.solve(z*np.eye(len(A))-A,B) for z in s])
 ax[0,i].semilogx(f,20*np.log10(abs(H)));ax[1,i].semilogx(f,np.unwrap(np.angle(H))*180/np.pi)
 ax[0,i].set_title(label);ax[0,i].set_ylabel("Magnitude (dB)");ax[1,i].set(ylabel="Phase (deg)",xlabel="Frequency (Hz)")
 for aa in ax[:,i]:
  aa.grid(alpha=.25);aa.axvline(.36,color="orange",ls="--");aa.axvline(2.54,color="red",ls=":")
fig.suptitle("Throttle plant with rotor dynamics and active steering: slow and fast coupled modes")
fig.savefig(a.output/"plant-frequency.png",dpi=160);plt.close(fig)
fig,ax=plt.subplots(1,2,figsize=(14,6),layout="constrained")
for gain in np.linspace(0,.25,151):
 cfg=dict(chosen,q=gain)
 ev=np.linalg.eigvals(close(A,B,C,cfg,22.2,.15)[0])
 for aa in ax:aa.scatter(ev.real,ev.imag,s=3,color=plt.cm.viridis(gain/.25),alpha=.45)
for name,marker,color in [("baseline","x","black"),("climb-2","o","red")]:
 ev=np.linalg.eigvals(nominal[name][3])
 for aa in ax:aa.scatter(ev.real,ev.imag,s=45,marker=marker,facecolors="none" if marker=="o" else color,edgecolors=color,label=name)
for aa in ax:aa.axvline(0,color="black");aa.grid(alpha=.25);aa.set(xlabel="Real part (1/s)",ylabel="Imaginary part (rad/s)");aa.legend()
ax[0].set(xlim=(-1.5,.2),ylim=(-.2,4),title="Altitude/integrator and slow pitch / forward-speed modes")
ax[1].set(xlim=(-4,1),ylim=(10,22),title="Fast coupled canopy–payload / lateral mode")
fig.suptitle("Pitch-damper gain locus 0 → 0.25 (purple → yellow); two 1.5 Hz stages, doubled climb PI")
fig.savefig(a.output/"root-locus.png",dpi=160);plt.close(fig)
# Individual loop breaks with the other feedback paths closed.
margin_rows=[]
fig,ax=plt.subplots(2,2,figsize=(14,9),layout="constrained")
for name in ["baseline","climb-2"]:
 t,y,r,ac,bc,cc,dd,L=nominal[name]
 cfg=next(c for c in configs if c["name"]==name)
 H=np.array([C@np.linalg.solve(z*np.eye(len(A))-A,B) for z in s])
 kv=.6/35*cfg["vm"];ki=.3*kv;V=kv+ki/s;F=(2*np.pi*cfg["hz"]/(s+2*np.pi*cfg["hz"]))**2
 damp=cfg["q"]*F*H[:,1];vertical=V*H[:,0];outer=(.15+V)*cfg["kh"]*H[:,0]/s
 loops={"complete throttle":L,"pitch damper":damp/(1+vertical+outer),"altitude outer":outer/(1+damp+vertical)}
 for label,loop in loops.items():
  phase=np.unwrap(np.angle(loop))*180/np.pi;db=20*np.log10(abs(loop))
  cross=crossings(np.log(f),db,0);pm=[180+np.interp(x,np.log(f),phase) for x in cross]
  gms=[]
  for angle in np.arange(-900,541,360):
   for x in crossings(np.log(f),phase,angle):gms.append(dict(frequency_hz=float(np.exp(x)),gain_margin_db=float(-np.interp(x,np.log(f),db))))
  margin_rows.append(dict(controller=name,loop=label,crossover_hz=np.exp(cross).tolist(),phase_at_crossings_deg=[float(np.interp(x,np.log(f),phase)) for x in cross],phase_distance_to_minus180_deg=[float((v+180)%360-180) for v in pm],gain_margins=gms,Ms=float(max(abs(1/(1+loop))))))
 ax[0,0].semilogx(f,20*np.log10(abs(L)),label=name);ax[1,0].semilogx(f,np.unwrap(np.angle(L))*180/np.pi,label=name)
 ax[0,1].semilogx(f,abs(1/(1+L)),label=name);ax[1,1].semilogx(f,abs(L/(1+L)),label=name)
for aa in ax.flat:aa.grid(alpha=.25);aa.legend();aa.set_xlabel("Frequency (Hz)")
ax[0,0].set_ylabel("Return ratio magnitude (dB)");ax[1,0].set_ylabel("Phase (deg)");ax[0,1].set_ylabel("|S|");ax[1,1].set_ylabel("|T|")
fig.suptitle("Complete throttle-loop response; coupled steering retained")
fig.savefig(a.output/"loop-frequency.png",dpi=160);plt.close(fig)
(a.output/"loop-margins.json").write_text(json.dumps(margin_rows,indent=2))
# Native-algebra nonlinear ramp comparison (when available).
ramp=a.output/"small-ramp"
if (ramp/"climb_x2.npz").exists():
 fig,ax=plt.subplots(3,1,figsize=(12,10),layout="constrained");validation=[]
 for lin,non in [("baseline","current"),("climb-2","climb_x2")]:
  t,y,r,*_=nominal[lin];d=np.load(ramp/(non+".npz"))["trace"];use=d[:,0]<=100
  for aa,li,ni,factor in zip(ax,[0,2,3],[1,4,5],[1,180/np.pi,1]):
   expected=np.interp(d[use,0],t,y[:,li]*factor)
   actual=d[use,ni]-(d[0,ni] if ni==5 else 0)
   aa.plot(d[use,0],actual,label=non+" nonlinear");aa.plot(d[use,0],expected,ls="--",label=lin+" linear")
   validation.append(dict(controller=lin,output_column=ni,rmse=float(np.sqrt(np.mean((actual-expected)**2))),peak_expected=float(max(abs(expected)))))
 for aa,label in zip(ax,["Altitude change (m)","Pitch rate (deg/s)","Throttle change"]):aa.set_ylabel(label);aa.grid(alpha=.25);aa.legend(fontsize=8)
 ax[-1].set_xlabel("Time (s)");fig.suptitle("Full-loop small-ramp check: continuous linear vs 50 Hz nonlinear diagnostic")
 fig.savefig(a.output/"full-loop-validation.png",dpi=160);plt.close(fig)
 (a.output/"full-loop-validation.json").write_text(json.dumps(validation,indent=2))
# Nyquist locus: multiple crossings and open-loop unstable modes preclude a
# single classical PM as a stability certificate. Closed-loop eigenvalues are definitive here.
fig,ax=plt.subplots(figsize=(8,7),layout="constrained")
for name in ["baseline","climb-2"]:
 L=nominal[name][-1]
 ax.plot(L.real,L.imag,label=name);ax.plot(L.real,-L.imag,ls="--",alpha=.5)
ax.scatter([-1],[0],color="red",marker="x",s=80,label="Critical point −1")
ax.set(xlim=(-3,2),ylim=(-3,3),xlabel="Real L",ylabel="Imaginary L",title="Complete throttle return ratio — positive / negative frequencies")
ax.grid(alpha=.25);ax.legend();fig.savefig(a.output/"nyquist.png",dpi=160);plt.close(fig)
(a.output/"open-loop-poles.json").write_text(json.dumps(dict(poles=[[float(z.real),float(z.imag)] for z in np.linalg.eigvals(A)],note="Throttle plant with steering closed. Altitude/climb integral factors add poles at the origin to the total return ratio; use closed-loop roots plus Nyquist, not a single PM."),indent=2))
# Uncertainty is applied to the rotor's local time scale, preserving its DC law.
uncertainty=[]
for file in sorted(a.plants.glob("*/plant.npz")):
 for scale in [.9,1.15,1.4]:
  for rotor_rate in [.7,1,1.3]:
   aa,bb,cc=coupled(file,scale);aa[14]*=rotor_rate;bb[14]*=rotor_rate
   for cfg in [configs[0],chosen]:
    voltage=20.4 if file.parent.name=="low_voltage" else 22.2
    slope=.18 if "descent" in file.parent.name else .15
    ev=np.linalg.eigvals(close(aa,bb,cc,cfg,voltage,slope)[0])
    uncertainty.append(dict(case=file.parent.name,scale=scale,rotor_rate=rotor_rate,controller=cfg["name"],max_real=float(ev.real.max())))
(a.output/"rotor-uncertainty.json").write_text(json.dumps(uncertainty,indent=2))
print("Rotor sensitivity checks",len(uncertainty),"unstable",sum(r["max_real"]>=0 for r in uncertainty))
# Transfer function represented without expanding ill-conditioned high-order polynomials.
transfer={}
for i,label in [(0,"climb_rate"),(1,"payload_pitch_rate")]:
 eig,V=np.linalg.eig(A);res=(C[i]@V)*(np.linalg.solve(V,B))
 transfer[label]=dict(form="G(s) = sum_j residue_j / (s - pole_j); throttle input, steering closed",
 poles=[[float(z.real),float(z.imag)] for z in eig],residues=[[float(z.real),float(z.imag)] for z in res])
(a.output/"transfer-functions.json").write_text(json.dumps(transfer,indent=2))
fig,ax=plt.subplots(1,2,figsize=(15,7),layout="constrained")
for aa,label in zip(ax,["climb","payload_pitch_rate"]):
 poles=np.linalg.eigvals(A);zz=np.array([complex(*v) for v in zeros[label]])
 aa.scatter(poles.real,poles.imag,marker="x",color="black",label="Plant poles")
 aa.scatter(zz.real,zz.imag,facecolors="none",edgecolors="tab:blue",label="Transfer zeros")
 aa.axvline(0,color="red",lw=1);aa.set(xlim=(-28,7),ylim=(-1,45),title=label.replace("_"," "),xlabel="Real part (1/s)",ylabel="Imaginary part (rad/s)")
 aa.annotate("Slow pitch / forward speed",xy=(-.307,2.152),xytext=(-23,4),arrowprops=dict(arrowstyle="->"),fontsize=10)
 aa.annotate("Coupled pitch / vertical motion",xy=(-.969,15.978),xytext=(-25,18),arrowprops=dict(arrowstyle="->"),fontsize=10)
 aa.annotate("Relative pitch + body rates",xy=(-5.117,25.402),xytext=(-25,28),arrowprops=dict(arrowstyle="->"),fontsize=10)
 aa.annotate("Roll / yaw coupling",xy=(-8.063,40.100),xytext=(-25,42),arrowprops=dict(arrowstyle="->"),fontsize=10)
 aa.grid(alpha=.25);aa.legend(loc="upper right",fontsize=9)
fig.suptitle("Throttle-to-output poles and zeros with steering closed; conjugate lower half omitted\nRotor pole −56.94 s⁻¹ and distant zeros lie outside this view. Zeros describe cancellation, not independent physical modes.",fontsize=13)
fig.savefig(a.output/"plant-poles-zeros.png",dpi=160);plt.close(fig)
# Numerically verify finite zeros using the resolvent away from poles.
checks={}
for i,label in [(0,"climb"),(1,"payload_pitch_rate")]:
 zs=[complex(*v) for v in zeros[label] if v[0]>0 and abs(complex(*v))<100]
 checks[label]=[dict(zero=[z.real,z.imag],absolute_response=float(abs(C[i]@np.linalg.solve(z*np.eye(len(A))-A,B)))) for z in zs]
(a.output/"zero-check.json").write_text(json.dumps(checks,indent=2))