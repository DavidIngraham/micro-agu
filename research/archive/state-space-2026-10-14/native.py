"""Run isolated parameter overrides against the existing native AUTO harness."""
import argparse
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path
from study import HERE, save

def main(repo, raw):
    cfg=json.loads((HERE/'optimization.json').read_text())['candidate']
    original=json.loads((repo/'Tools/simulation/paraglider/mission-controller-candidate.json').read_text())
    candidate=json.loads(json.dumps(original));par=candidate['parameters']
    par.update(PG_TURN_P=cfg['kp'],PG_TURN_I=cfg['ki'],PG_TURN_RDAMP=cfg['rd'],
               PG_TURN_FILT=cfg['lat_hz'],TECS_PTCH_DAMP=cfg['qgain'],
               TECS_PG_PR_FILT=cfg['qhz'],TECS_PG_PR_FILT2=cfg['qhz'],
               TECS_VERT_ACC=.6/(5*cfg['kv']),TECS_HDEM_TCONST=1/cfg['kh'])
    candidate['description']='Isolated state-space study candidate; not a promoted default.'
    save('native-baseline.json',original);save('native-candidate.json',candidate)
    tools=HERE.parents[1]/'tools'
    env=os.environ.copy();env['PATH']=str(Path(sys.executable).parent)+os.pathsep+env['PATH']
    results=[]
    for condition in ['gust','calm']:
        for name in ['baseline','candidate']:
            out=raw/f'{name}-{condition}'
            if out.exists():raise RuntimeError(f'Refusing to overwrite {out}')
            out.mkdir(parents=True)
            with (out/'driver.log').open('w') as log:
                proc=subprocess.run(['bash',str(tools/'run_gust_native.sh'),str(repo),str(out),str(HERE/f'native-{name}.json'),condition],
                                    cwd=repo,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=900)
            record=dict(controller=name,condition=condition,returncode=proc.returncode,raw_directory=str(out))
            if proc.returncode==0:
                result=subprocess.run([sys.executable,str(tools/'summarize_gust_mission.py'),str(out)],capture_output=True,text=True)
                (out/'summary.log').write_text(result.stdout+result.stderr)
                if result.returncode==0:record['metrics']=json.loads((out/'gust-metrics.json').read_text())
                else:record['summary_error']=result.stderr[-2000:]
            else:
                record['error_tail']=(out/'run.log').read_text(errors='replace')[-4000:] if (out/'run.log').exists() else (out/'driver.log').read_text()[-4000:]
            results.append(record);save('native-results.json',results);print(name,condition,proc.returncode,flush=True)
    save('native-provenance.json',dict(binary_sha256=hashlib.sha256((repo/'build/sitl/bin/arduplane').read_bytes()).hexdigest(),
                                      checkout_revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=repo,text=True).strip(),
                                      note='Checkout revision is not a claim about the executable build tree. See native-build-audit.json for each BIN version record.'))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--raw',type=Path,required=True)
    a=p.parse_args();main(a.repo,a.raw)
