"""AI-assisted constrained nonlinear steady-turn equilibria, as-built physics."""
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
spec=importlib.util.spec_from_file_location('design',ROOT/'research/archive/legacy-control/design_mimo.py')
d=importlib.util.module_from_spec(spec);spec.loader.exec_module(d)

def main():
    sim=d.make_simulator('trout_lake','as_built');sim.substeps=16
    state,u,rpm,res=d.trim(sim,common=.1);chart=d.Chart(sim,state,rpm)
    np.savez(HERE/'base.npz',state=state,u=u,rpm=rpm)
    def advance(x,control):
        y,rotor=chart.decode(x*d.XS);actuator=np.clip(d.command(control),0,np.r_[1,d.STROKES]);y,rotor=sim.step(y,rotor,actuator,dt=d.DT)
        return chart.encode(y,rotor)/d.XS
    def outputs(x):
        y,_=chart.decode(x*d.XS);roll,pitch=Rotation.from_quat(y[3:7]).as_euler('xyz')[:2]
        rates=chart.payload_rates+x[8:11]*d.XS[8:11]
        return (rates[1]*np.sin(roll)+rates[2]*np.cos(roll))/np.cos(pitch),np.linalg.norm(y[7:10])
    records=[];solutions={}
    for rate in [0,10,-10,15,-15,20,-20,25,-25,30,-30,40,-40,50,-50,60,-60]:
        last=None
        for speed in [4.2,4.0,3.8,4.3,4.4]:
            target=np.deg2rad(rate)
            def residual(v):
                x=np.r_[v[:15],0];control=v[15:]
                rr,vv=outputs(x)
                return np.r_[(advance(x,control)-x)/d.DT,(rr-target)/np.deg2rad(10),(vv-speed)/.5,
                             max(0,control[1]+abs(control[2])-1)*100]
            initial=last if last is not None else np.r_[np.zeros(15),u]
            if last is None and solutions:
                key=min(solutions,key=lambda k:abs(k[0]-rate)/10+abs(k[1]-speed)/.2)
                initial=solutions[key].copy()
            fit=least_squares(residual,initial,bounds=(np.r_[np.full(14,-10),-rpm/1000,0,0,-1],np.r_[np.full(14,10),10,.6,1,1]),max_nfev=220,ftol=1e-10,xtol=1e-10,gtol=1e-10,x_scale='jac')
            error=residual(fit.x);feasible=bool(fit.success and np.max(abs(error))<1e-3)
            x=np.r_[fit.x[:15],0];control=fit.x[15:];rr,vv=outputs(x)
            row=dict(rate=rate,speed=speed,feasible=feasible,residual=float(np.max(abs(error))),actual_rate=float(np.rad2deg(rr)),actual_speed=float(vv),x=x.tolist(),u=control.tolist(),nfev=fit.nfev)
            if feasible:
                solutions[rate,speed]=fit.x;last=fit.x
                a=d.jacobian(lambda xx:advance(xx,control),x)
                # One-sided common derivative at the nonnegative boundary.
                b=np.column_stack([(advance(x,control+delta)-advance(x,control))/1e-5 if i==1 and control[1]<1e-4 else (advance(x,control+delta)-advance(x,control-delta))/2e-5 for i,delta in enumerate(np.eye(3)*1e-5)])*.1
                np.savez(HERE/f'trim-{rate}-{speed:.1f}.npz',x=x,u=control,A=a,B=b)
            records.append(row);print(json.dumps({k:row[k] for k in ['rate','speed','feasible','residual','u','nfev']}),flush=True)
            (HERE/'trims.json').write_text(json.dumps(records,indent=2))
if __name__=='__main__':main()
