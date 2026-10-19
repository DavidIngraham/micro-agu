
"""Straight-level equilibria and fixed-command perturbations for each mechanism."""
import concurrent.futures as cf,json,subprocess
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb
from microagu_sim.compat import dynamics
from microagu_control.pipeline.linearize import trim,command
from run import OUT,BIN,execute,rest_lengths,initialize_risers

def case(flight,label,options):
    sim=make_simulator(flight,profile="as_built")
    seed,u0,_,_=trim(sim,common=.1)
    e=options.copy();free=bool(e.get("suspension_k"))
    if free:
        e["rest_lengths"]=rest_lengths(sim,e["suspension_k"])
        seed=initialize_risers(seed,sim,e)
    process=subprocess.Popen([str(BIN)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def native(req,field):
        process.stdin.write(json.dumps(req)+chr(10));process.stdin.flush()
        reply=json.loads(process.stdout.readline())
        if "error" in reply:raise RuntimeError(reply["error"])
        return np.array(reply[field])
    fixed=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],
        wind=[0,0,0],voltage=22.2,experiment=e)
    def unpack(x):
        y=perturb(seed,x[:15],sim.geometry)
        # perturb() reconstructs the point geometry; restore compliant separation.
        if free:
            base_point=perturb(seed,np.zeros(15),sim.geometry)
            y[13:16]+=seed[13:16]-base_point[13:16]+x[15:18]
            y[20:23]+=seed[20:23]-base_point[20:23]+x[18:21]
        u=np.array([x[-2],.1,x[-1]])
        rpm=743.5778222336925*22.2*max(u[0]-.0005141016498671315,0)**.9086807307118114
        return y,u,rpm
    def residual(x):
        y,u,rpm=unpack(x)
        dy=native(fixed|dict(state=y.tolist(),commands=[sim.controls(*command(u)).tolist()],
            rpm=rpm,derivative=True),"derivative")
        res=np.r_[y[10:13],y[23:26],dy[7:10],dy[10:13],dy[23:26]]
        if free:res=np.r_[res,y[20:23]-y[7:10],dy[20:23]]
        return np.r_[res,y[9],Rotation.from_quat(y[3:7]).as_euler("xyz")[2]]
    n=21 if free else 15
    lo=np.r_[np.full(6,-1.),np.full(9,-3.),np.full(3,-.3),np.full(3,-3.)] if free else np.r_[np.full(6,-1.),np.full(9,-3.)]
    hi=-lo
    try:
        fit=least_squares(residual,np.r_[np.zeros(n),u0[0],u0[2]],bounds=(np.r_[lo,.01,-.6],np.r_[hi,.8,.6]),
            max_nfev=250,ftol=1e-11,xtol=1e-11,gtol=1e-10)
        err=float(abs(residual(fit.x)).max())
        if not fit.success or err>1e-6:raise RuntimeError(f"trim: {fit.message}; residual={err}")
        y,u,rpm=unpack(fit.x)
        base=None;results=[]
        for angle in [0,1,5,15]:
            z=y.copy();rc=Rotation.from_quat(z[16:20])
            z[16:20]=(rc*Rotation.from_euler("y",angle,degrees=True)).as_quat()
            if not free:
                rp=Rotation.from_quat(z[3:7]);rc=Rotation.from_quat(z[16:20])
                z[13:16]=z[:3]+rp.apply(sim.geometry["hinge"])-rc.apply([0,0,.85])
            req=fixed|dict(state=z.tolist(),commands=[sim.controls(*command(u)).tolist()]*1001,rpm=rpm,substeps=32)
            rows=native(req,"rows")
            rel=np.rad2deg((Rotation.from_quat(rows[:,3:7]).inv()*Rotation.from_quat(rows[:,16:20])).as_rotvec())
            if angle==0:base=rel
            delta=rel-base
            early=float(np.sqrt(np.mean(delta[25:125]**2)));late=float(np.sqrt(np.mean(delta[750:]**2)))
            results.append(dict(flight=flight,variant=label,amplitude=angle,trim_residual=err,
                trim_command=u.tolist(),early_rms_deg=early,late_rms_deg=late,ratio=late/max(early,1e-12),
                max_relative_angle_deg=float(abs(delta).max())))
            np.savez_compressed(OUT/"traces"/f"constant-{flight}-{label}-{angle}.npz",rows=rows,delta=delta)
        return results
    finally:
        process.stdin.close();process.wait(timeout=10)
if __name__=="__main__":
    variants=json.loads((OUT/"protocol.json").read_text())["variants"]
    def safe(args):
        try:return case(*args)
        except Exception as error:return [dict(flight=args[0],variant=args[1],error=str(error))]
    jobs=[(f,k,v) for f in ["hood_river","trout_lake"] for k,v in variants.items()]
    with cf.ThreadPoolExecutor(max_workers=4) as pool:
        results=[r for batch in pool.map(safe,jobs) for r in batch]
    (OUT/"constant-results.json").write_text(json.dumps(results,indent=2)+chr(10))
    print("records",len(results),"failed trims",sum("error" in r for r in results))
