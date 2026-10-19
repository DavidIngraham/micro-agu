
"""Independently trimmed constant-command perturbations: not flight-data fitting."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_sim.compat import dynamics
from microagu_sim.compat.profiles import make_simulator
from microagu_control.pipeline.linearize import trim, command, Chart
from run import OUT, execute

def relative(rows):
    rp=Rotation.from_quat(rows[:,3:7])
    rc=Rotation.from_quat(rows[:,16:20])
    return np.rad2deg((rp.inv()*rc).as_rotvec())

def case(flight,label,options,common,fitted):
    sim=make_simulator(flight,profile="as_built")
    family="fitted" if fitted else "prior"
    if fitted:
        from microagu_control.sysid.aircraft import make_aircraft,snapshot
        theta=json.loads((OUT.parent/"hood-river-pipeline-2026-10-18/fit/fit.json").read_text())["selection"]["theta"]
        sim=make_aircraft(snapshot(flight),theta)
        # Constant servo commands start at steady actuator pull, so the fitted
        # command lag has no transient in this perturbation experiment.
    def advance(state,rpm,commands,voltage=22.2,dt=.004):
        request=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],
            state=state.tolist(),commands=[sim.controls(*commands).tolist()]*2,
            wind=[0,0,0],voltage=voltage,rpm=rpm,dt=dt,substeps=16)|options
        rows=execute(request)
        return rows[-1,:26],rows[-1,26]
    sim.step=advance
    state,u,rpm,residual=trim(sim,common=common)
    chart=Chart(sim,state,rpm)
    output=[]
    base=None
    for amplitude in [0,1,5,15]:
        x=np.zeros(16);x[3]=np.deg2rad(amplitude)
        y,_=chart.decode(x)
        request=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry["mass"],
            state=y.tolist(),commands=[sim.controls(*command(u)).tolist()]*1001,
            wind=[0,0,0],voltage=22.2,rpm=rpm,substeps=32)|options
        if options.get("frozen"):
            request["frozen_reference"]=dynamics.brake_paths(state,sim.parameters).tolist()
        rows=execute(request)
        angles=relative(rows)
        if amplitude==0:base=angles
        delta=angles-base
        early=float(np.sqrt(np.mean(delta[25:125]**2)))
        late=float(np.sqrt(np.mean(delta[750:]**2)))
        result=dict(flight=flight,variant=label,common=common,family=family,amplitude_deg=amplitude,trim_residual=residual,
            command=u.tolist(),early_relative_angle_rms_deg=early,late_relative_angle_rms_deg=late,
            late_over_early=late/max(early,1e-12),max_relative_angle_deg=float(np.max(abs(delta))))
        output.append(result)
        np.savez_compressed(OUT/"traces"/f"constant-{flight}-{family}-{common}-{label}-{amplitude}.npz",rows=rows,angles=angles,delta=delta)
    return output

if __name__=="__main__":
    import concurrent.futures as cf
    variants=json.loads((OUT/"protocol.json").read_text())["variants"]
    jobs=[(f,k,v,c,fit) for f,fit in [("hood_river",False),("trout_lake",False),("hood_river",True)] for c in [.02,.1,.3] for k,v in variants.items()]
    def safe(args):
        try:return case(*args)
        except Exception as e:return [dict(flight=args[0],variant=args[1],common=args[3],family="fitted" if args[4] else "prior",error=str(e))]
    with cf.ThreadPoolExecutor(max_workers=4) as pool:
        results=[r for batch in pool.map(safe,jobs) for r in batch]
    (OUT/"constant-results.json").write_text(json.dumps(results,indent=2)+chr(10))
    print("constant runs",len(results),"errors",sum("error" in x for x in results))
