import json
import numpy as np
from study import Backend,load,configured_simulator,initialize,dynamics,OUT
b=Backend();s=load()[0];sim=configured_simulator(s);s.rpm=0.;y=initialize(sim,s,.35,s.wind);u=s.commands[:101].copy();u[:,0]=0.;native=np.array([sim.controls(*a) for a in u])
r=dict(parameters=dynamics.params(sim.parameters).tolist(),mass=sim.geometry['mass'],state=y.tolist(),commands=native.tolist(),rpm=np.zeros(len(u)).tolist(),wind=s.wind.tolist(),substeps=32,power_damping=0.)
a=b.call(r);c,_=sim.replay(y,u,s.voltage,0.,s.wind);error=float(np.max(abs(a-c)));assert error<1e-8
r['substeps']=64;fine=b.call(r);resolution=float(np.max(abs(a-fine)));assert resolution<1e-5
# Constant nonzero RPM agrees with production when its target equals that RPM.
from microagu_sim import _native
rpm=743.5778222336925*s.voltage*(.4-.0005141016498671315)**.9086807307118114
u[:,0]=.4;r['commands']=[sim.controls(*v).tolist() for v in u];r['rpm']=[rpm]*len(u);r['substeps']=32
a=b.call(r);c,_=sim.replay(y,u,s.voltage,rpm,s.wind);powered=float(np.max(abs(a-c)));assert powered<1e-7
(OUT/'backend-checks.json').write_text(json.dumps(dict(zero_rpm_parity=error,zero_rpm_resolution=resolution,constant_powered_rpm_parity=powered),indent=2));print(error,resolution,powered);b.proc.terminate()
