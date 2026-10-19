import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
from run import apply,variants,OUT
from trim import solve

def analyze(flight,label,options,h=.004):
    sim=apply(make_simulator(flight,profile="as_built",substeps=32),options)
    seed=np.load(Path(compat.__file__).parent/f"equilibrium-trim-{flight}-0.0-0.0.npz")["state"]
    fit=solve(sim,seed,None)
    residual=float(max(abs(fit["residual"])))
    row=dict(flight=flight,variant=label,dt=h,trim_residual=residual)
    if residual>1e-4:return row
    y,u,rpm=fit["state"],fit["command"],fit["rpm"]
    base,_=sim.step(y,rpm,u,dt=h)
    cols=[]
    for k in range(15):
        x=np.eye(15)[k]*1e-5
        plus,_=sim.step(perturb(y,x,sim.geometry),rpm,u,dt=h)
        minus,_=sim.step(perturb(y,-x,sim.geometry),rpm,u,dt=h)
        cols.append((difference(base,plus)-difference(base,minus))/2e-5)
    jac=np.column_stack(cols)
    val,vec=np.linalg.eig(jac)
    lam=np.log(val.astype(complex))/h
    row["max_real_eigenvalue"]=float(max(lam.real))
    modes=[]
    for j,v in enumerate(lam):
        if v.imag<=.1:continue
        z=vec[9:12,j]
        modes.append(dict(hz=float(v.imag/2/np.pi),decay=float(-v.real),damping=float(-v.real/abs(v)),pitch_yaw_ratio=float(abs(z[1])/max(abs(z[2]),1e-12)),phase=float(np.angle(z[1]*z[2].conjugate(),deg=True))))
    row["modes"]=modes
    up=u.copy();um=u.copy()
    up[2]+=.005;up[1]-=.005;um[2]-=.005;um[1]+=.005
    yp,_=sim.step(y,rpm,up,dt=h);ym,_=sim.step(y,rpm,um,dt=h)
    b=(difference(base,yp)-difference(base,ym))/.02
    z=np.linalg.solve(np.exp(2j*np.pi*1.5625*h)*np.eye(15)-jac,b)[9:12]
    row["forced"]=dict(yaw_gain=float(abs(z[2])*180/np.pi),ratio=float(abs(z[1]/z[2])),phase=float(np.angle(z[1]/z[2],deg=True)))
    if h==.004:
        decay=[]
        for deg in [0.,1.,5.,15.]:
            x=np.zeros(15);x[4]=np.deg2rad(deg)
            ys,rs=sim.replay(perturb(y,x,sim.geometry),np.tile(u,(1001,1)),22.2,rpm)
            rel=(Rotation.from_quat(ys[:,3:7]).inv()*Rotation.from_quat(ys[:,16:20])).as_rotvec()
            eq=(Rotation.from_quat(y[3:7]).inv()*Rotation.from_quat(y[16:20])).as_rotvec()
            amplitude=np.linalg.norm(rel-eq,axis=1)*180/np.pi
            early=float(np.sqrt(np.mean(amplitude[25:250]**2)))
            late=float(np.sqrt(np.mean(amplitude[-250:]**2)))
            decay.append(dict(perturb_deg=deg,early_rms=early,late_rms=late,ratio=late/max(early,1e-12)))
        row["decay"]=decay
    return row
if __name__=="__main__":
    v=variants()
    labels=["baseline","J4.0-C0.25-D0.1","J4.0-C1.0-D0.1","offset0.0","offset0.75","payload0.7","payload1.3","coupled-offset0.0-D0.1","coupled-offset0.75-D0.1","pitch-offset-0.5","pitch-offset0.5","coupled-pitch-offset-0.5","coupled-pitch-offset0.5"]
    rows=[]
    for flight in ["hood_river","trout_lake"]:
        for label in labels:
            try:r=analyze(flight,label,v[label])
            except Exception as e:r=dict(flight=flight,variant=label,error=str(e))
            rows.append(r);print(flight,label,r.get("trim_residual",r.get("error")),flush=True)
        for label in ["baseline","J4.0-C0.25-D0.1"]:
            try:rows.append(analyze(flight,label,v[label],.002))
            except Exception as e:rows.append(dict(flight=flight,variant=label,dt=.002,error=str(e)))
    (OUT/"modal-results.json").write_text(json.dumps(rows,indent=2))
