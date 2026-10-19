import json,subprocess
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb,difference
from microagu_sim.compat import dynamics
from run import OUT,BIN,variants
from trim import solve
def analyze(flight,label):
    sim=make_simulator(flight,profile="as_built")
    seed=np.load(Path(compat.__file__).parent/f"equilibrium-trim-{flight}-0.0-0.0.npz")["state"]
    base=solve(sim,seed,None)
    y0=base["state"];u0=base["command"]
    e=variants()[label]
    req=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],wind=[0.,0.,0.],voltage=22.2,experiment=e,substeps=32)
    process=subprocess.Popen([str(BIN)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
    def execute(q):
        process.stdin.write(json.dumps(q)+chr(10));process.stdin.flush()
        r=json.loads(process.stdout.readline())
        if "error" in r:raise ValueError(r["error"])
        return np.array(r["rows"])
    def unpack(x):
        y=perturb(y0,x[:15],sim.geometry)
        u=u0+np.array([x[-2],-x[-1]/2,x[-1]/2])
        rpm=743.5778222336925*22.2*max(u[0]-.0005141016498671315,0)**.9086807307118114
        return y,u,rpm
    def residual(x):
        y,u,rpm=unpack(x)
        n=execute(req|dict(state=y.tolist(),rpm=rpm,commands=[sim.controls(*u).tolist()]*2,twist=x[15:17].tolist(),dt=.004))[-1]
        return np.r_[difference(y,n[:26])/.004,(n[26:28]-x[15:17])/.004,y[9],Rotation.from_quat(y[3:7]).as_euler("xyz")[2]]
    try:
        x=np.zeros(19)
        if e["enabled"]:
            lo=np.r_[np.full(6,-.7),np.full(3,-3.),np.full(6,-3.),-.35,-2.,-u0[0]+.01,-1.98*(u0[2]-sim.parameters["right_offset"]*1000)]
            hi=np.r_[np.full(6,.7),np.full(3,3.),np.full(6,3.),.35,2.,.9-u0[0],1.98*(u0[1]-sim.parameters["left_offset"]*1000)]
            fit=least_squares(residual,x,bounds=(lo,hi),max_nfev=120,xtol=1e-10,ftol=1e-10,gtol=1e-9)
            x=fit.x
        res=float(max(abs(residual(x))))
        y,u,rpm=unpack(x)
        row=dict(flight=flight,variant=label,trim_residual=res,trim_twist_deg=float(np.rad2deg(x[15])),command=u.tolist())
        if res>1e-4:return row
        # Rigid states plus twist and twist-rate; RPM perturbations excluded.
        h=.004;eps=1e-5
        refrow=execute(req|dict(state=y.tolist(),rpm=rpm,commands=[sim.controls(*u).tolist()]*2,twist=x[15:17].tolist(),dt=h))[-1]
        cols=[]
        for j in range(17):
            dx=np.eye(17)[j]*eps
            plus=execute(req|dict(state=perturb(y,dx[:15],sim.geometry).tolist(),rpm=rpm,commands=[sim.controls(*u).tolist()]*2,twist=(x[15:17]+dx[15:17]).tolist(),dt=h))[-1]
            minus=execute(req|dict(state=perturb(y,-dx[:15],sim.geometry).tolist(),rpm=rpm,commands=[sim.controls(*u).tolist()]*2,twist=(x[15:17]-dx[15:17]).tolist(),dt=h))[-1]
            cols.append(np.r_[difference(refrow[:26],plus[:26])-difference(refrow[:26],minus[:26]),plus[26:28]-minus[26:28]]/(2*eps))
        lam=np.log(np.linalg.eigvals(np.column_stack(cols)).astype(complex))/h
        row["eigenvalues"]=[dict(real=float(v.real),hz=float(v.imag/(2*np.pi))) for v in lam]
        cases=[]
        for kind,deg in [("twist",0),("twist",1),("twist",5),("twist",15),("pitch",5)]:
            if not e["enabled"] and kind=="twist" and deg>0:continue
            z=y.copy();tw=x[15:17].copy()
            if kind=="twist":tw[0]+=np.deg2rad(deg)
            else:
                dx=np.zeros(15);dx[4]=np.deg2rad(deg);z=perturb(y,dx,sim.geometry)
            rows=execute(req|dict(state=z.tolist(),rpm=rpm,commands=[sim.controls(*u).tolist()]*1001,twist=tw.tolist(),dt=.04))
            relative=Rotation.from_quat(rows[:,3:7]).inv()*Rotation.from_quat(rows[:,16:20])
            ref=Rotation.from_quat(y[3:7]).inv()*Rotation.from_quat(y[16:20])
            ang=(ref.inv()*relative).magnitude()
            amplitude=np.rad2deg(np.sqrt(ang**2+(rows[:,26]-x[15])**2))
            early=float(np.sqrt(np.mean(amplitude[25:250]**2)));late=float(np.sqrt(np.mean(amplitude[-250:]**2)))
            cases.append(dict(kind=kind,degrees=deg,early=early,late=late,ratio=late/max(early,1e-10),peak_twist=float(np.rad2deg(abs(rows[:,26]).max()))))
        row["decay"]=cases
        return row
    finally:
        process.stdin.close();process.wait()
if __name__=="__main__":
    labels=json.loads((OUT/"selected-diagnostics.json").read_text())["labels"]
    rows=[]
    for flight in ["hood_river","trout_lake"]:
        for label in labels:
            try:row=analyze(flight,label)
            except Exception as e:row=dict(flight=flight,variant=label,error=str(e))
            rows.append(row);print(row,flush=True)
    (OUT/"constant-results.json").write_text(json.dumps(rows,indent=2))
