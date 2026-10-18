"""AI-assisted predeclared stepwise envelope screening with fixed gains."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from build import HERE,VIEW,ENV,sha
from score import assess

def specs(heldout=False):
    rng=np.random.default_rng(2026101802 if heldout else 2026101801)
    cases=[]
    for i in range(12 if heldout else 8):
        nominal=not heldout and i<2
        cases.append(dict(id=i+(100 if heldout else 0),sign=1 if i%2==0 else -1,
            scales={} if nominal else {k:float(rng.uniform(.7,1.3)) for k in ['Cmq','joint_c_pitch','joint_c_roll','joint_c_yaw','CLbrake','CDbrake']},
            brakeRateScale=1 if nominal else float(rng.uniform(.7,1.1)),wind=0 if nominal else float(rng.uniform(0,2)),
            turbulence=0 if nominal else float(rng.uniform(.1,.6)),seed=int(rng.integers(1,2**32-1))))
    return cases

def batch(rate,cases,phase='screen',scheme='lqi'):
    out=HERE/phase/f'{scheme}-{rate:02}';out.mkdir(parents=True,exist_ok=True)
    def run(s):
        s=dict(s,rate=rate,scheme=scheme,runtime=str(HERE/'runtime'));p=out/f'{s["id"]:03}.json';sp=out/f'{s["id"]:03}-spec.json'
        if not p.exists():
            sp.write_text(json.dumps(s,indent=2))
            with p.with_suffix('.log').open('w') as log:
                subprocess.run([str(VIEW/'node_modules/.bin/tsx'),'--tsconfig',str(VIEW/'tsconfig.json'),str(HERE/'case.mts'),str(sp),str(p)],cwd=VIEW,env=ENV,stdout=log,stderr=subprocess.STDOUT,timeout=400)
        if not p.exists():raise RuntimeError(str(p.with_suffix('.log')))
        result=assess(p);p.with_name(p.stem+'-metrics.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);return result
    with ThreadPoolExecutor(max_workers=3) as pool:results=list(pool.map(run,cases))
    (out/'summary.json').write_text(json.dumps(results,indent=2));return results

def main():
    protocol=dict(rates=[10,15,20,25,30,35,40,45,50,55,60],training=specs(),heldout=specs(True),
        fixed='Published responsive_softspeed K/AW/observer/model; HRTC=.7; rate acceleration40 deg/s²; original sensing; normal full-stick CRUISE path',
        gates='See score.py frozen before runs. Require all nominal and uncertainty cases to pass before expansion. Stop at first failed level; explore failures separately without promoting them.',
        firmware_sha256=sha(HERE/'runtime/arduplane.wasm'))
    if not (HERE/'protocol.json').exists():(HERE/'protocol.json').write_text(json.dumps(protocol,indent=2))
    for rate in protocol['rates']:
        results=batch(rate,protocol['training'])
        if not all(r['passed'] for r in results):
            print('STOP AT '+str(rate),flush=True);break
if __name__=='__main__':main()
