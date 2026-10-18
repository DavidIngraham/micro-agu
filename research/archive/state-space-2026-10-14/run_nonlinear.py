"""Parallel numerical workers (four independent controller runs), then merge."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
from study import HERE, save
from nonlinear import plot

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);a=p.parse_args()
    jobs=[]
    for name in ['baseline','candidate','effort','oracle_selected']:
        log=(HERE/f'nonlinear-{name}.log').open('w')
        jobs.append((name,subprocess.Popen([sys.executable,str(HERE/'nonlinear.py'),'--model',str(a.model),'--controller',name],stdout=log,stderr=subprocess.STDOUT),log))
    for name,job,log in jobs:
        code=job.wait();log.close()
        if code:raise RuntimeError(f'{name}: exit {code}; inspect its log')
    rows=[];validation=[]
    for name,_,_ in jobs:
        rows.extend(json.loads((HERE/f'nonlinear-summary-{name}.json').read_text()))
        validation.extend(json.loads((HERE/f'linear-validation-{name}.json').read_text())['rows'])
    save('nonlinear-summary.json',rows)
    save('linear-validation.json',dict(outputs=['height','climb','payload q','throttle','brake'],rows=validation,
         note='1 mm ramp, smooth feedforward branch; nonlinear physics and travel-rate actuator included.'))
    plot()
