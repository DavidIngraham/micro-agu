import numpy as np
from analyze import target
from microagu_sim.compat.profiles import make_simulator
sim=make_simulator("trout_lake",profile="as_built")
for throttle,voltage in [(.2,20),(.5,22.2),(.6,24)]:
    y,r=sim.step(sim.initial_state(),4000,[throttle,0,0],voltage=voltage,dt=.04)
    expected=target(throttle,voltage)+(4000-target(throttle,voltage))*np.exp(-.04/.01756125311487475)
    assert abs(r-expected)<1e-8
print("Analytic RPM propagation matches native Rust for three throttle/voltage cases")
