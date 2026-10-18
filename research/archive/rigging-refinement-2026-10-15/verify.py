# AP_FLAKE8_CLEAN
"""Geometry invariants, exact disabled-mode parity and integration convergence."""
import json,sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from study import HERE,make,replay
from microagu_sim.compat import dynamics,routing
# Independent disabled-mode output from the historical kernel.
from microagu_sim import Kernel
config=json.loads((HERE/'disabled-reference.json').read_text())
assert config['parameters'][60]==0
kernel=Kernel(config['parameters'],config['mass'])
np.testing.assert_allclose(kernel.derivative(config['state'],config['command'],config['wind']),config['derivative'],rtol=1e-10,atol=1e-10)
s,trim=make('trout_lake');y=trim['state']
routing.configure(s,y)
assert np.max(abs(routing.diagnostic(s,y,trim['command'])['delta_m']))<1e-14
# A common world rotation leaves relative geometry unchanged.
z=y.copy();q=R.from_euler('xyz',[.5,-.2,1.])
for k in [3,16]:z[k:k+4]=(q*R.from_quat(y[k:k+4])).as_quat()
assert np.max(abs(dynamics.brake_paths(z,s.parameters)-dynamics.brake_paths(y,s.parameters)))<1e-14
# Mirror-symmetric attitudes produce equal left/right paths.
z=s.initial_state();assert abs(np.diff(dynamics.brake_paths(z,s.parameters))[0])<1e-14
# Zero ring height removes pitch-only relative motion along the riser-bolt axis.
s.parameters['ring_height']=0.;base=dynamics.brake_paths(z,s.parameters)
z[3:7]=R.from_euler('y',.4).as_quat()
assert np.max(abs(dynamics.brake_paths(z,s.parameters)-base))<1e-14
# A manually increased path consumes slack before brake engagement.
s.parameters['routing_enabled']=1.;s.parameters['left_offset']=.012
assert s.controls(.3,0,0)[1]==-.012
routing.configure(s,z)
s.parameters['route_reference_left']-=.020
assert abs(routing.diagnostic(s,z,[.3,0,0])['effective_m'][0]-.008)<1e-12
geometry_only='--geometry-only' in sys.argv
rows=[] if geometry_only else [replay(('trout_lake',840,'routed',(1.,1.,1.),30,.35,.06,n)) for n in [32,64]]
relative={} if geometry_only else {k:abs(rows[0]['metrics'][k]-rows[1]['metrics'][k])/max(rows[1]['metrics'][k],1e-8) for k in rows[0]['metrics']}
assert max(relative.values(),default=0)<.001
result=dict(disabled_derivative_reference_tolerance=1e-10,geometry_invariants=True,slack_before_clip=True,integration_relative_metric_difference=relative)
if not geometry_only:(HERE/'verification.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))
