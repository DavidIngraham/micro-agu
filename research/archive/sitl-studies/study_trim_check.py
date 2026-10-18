from _sim_path import SIMULATION
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.trim import find_trim
import json
s=make_simulator('trout_lake','mission')
for c in [-.4,0,.4,.8,1.]:
 t=find_trim(s,climb_m_s=c);print(json.dumps({'climb':c,'command':t['command'].tolist()}),flush=True)
