import json
import numpy as np
import experiment as ex
from iterate import Shaped,closure,stress

def main():
    choice=json.loads((ex.HERE/'iteration2/refined-codesign-selection.json').read_text())
    variant=choice['selected']['variant'];Shaped.variant=variant
    grid=dict(np.load(ex.HERE/'grid.npz'));K=np.load(ex.HERE/'gains.npz')['K']
    records=json.loads((ex.HERE/'grid.json').read_text())['nodes']
    sim=ex.setup(ex.MODEL)('trout_lake','mission');sim.substeps=16
    p=ex.plants()[0];chart=ex.Chart(sim,p['state'],float(p['rpm']))
    checks=[]
    for i in [4,9,13,17]:
        r=records[i];x0=grid['x'][i];u0=grid['u'][i]
        A,B,U=closure(grid,K,i,variant);F=A+B@U
        def step(z):
            ctrl=Shaped(grid,K);ctrl.integral=z[16:18].copy();ctrl.qfilters=z[18:].copy()
            x=x0+ex.XS*z[:16];y,rpm=chart.decode(x)
            raw=ctrl.request(x,ex.observations(y),0,r['climb'],r['turn'],r['voltage'],r['speed'])
            act=ex.Actuation(sim,np.r_[u0[0],ex.brake_command(u0[1])]);cmd,applied=act.apply(raw,r['voltage']);ctrl.applied(applied)
            yn,rn=sim.step(y,rpm,cmd,voltage=r['voltage'],dt=ex.DT)
            xx=chart.encode(yn,rn)-x0;xx[15]-=r['climb']*ex.DT
            return np.r_[xx/ex.XS,ctrl.integral,ctrl.qfilters]
        eps=1e-6
        actual=np.column_stack([(step(e)-step(-e))/(2*eps) for e in np.eye(20)*eps])
        error=float(np.linalg.norm(actual-F)/np.linalg.norm(F));checks.append(dict(node=i,error=error));assert error<1e-5
    ex.save('iteration2/final-verification.json',dict(variant=variant,closure=checks,stress=stress(variant)))
    print(checks)

if __name__=='__main__':main()
