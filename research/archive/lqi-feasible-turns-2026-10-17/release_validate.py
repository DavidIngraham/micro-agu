"""Fresh expanded-envelope release gate, frozen before execution."""
import json
from concurrent.futures import ThreadPoolExecutor
from run import batch,specs,HERE
from build import sha

def main():
    cases=[dict(s,weatherOffRecovery=True) for s in specs(True)[6:]]
    candidate={'CP3_AWMODE':1,'CP3_TMAP':1,'CP3_OBSTRIM':1}
    plan=dict(cases=cases,candidate=candidate,rates=[25,40],
        firmware_sha256=sha(HERE/'runtime/arduplane.wasm'),
        gates='Retain every original score.py gate. All candidate cases must pass before production integration or push. Weather removed only at t=200; final ten seconds independently assess unforced recovery. Failures block release; do not relax thresholds after results. If disturbance gate passes, proceed to transition checks and production command/table bounds.',
        baseline='Matched reference at 25 deg/s for attribution, not a waiver of candidate failures.')
    path=HERE/'release-validation-protocol.json'
    if path.exists():assert json.loads(path.read_text())==plan
    else:path.write_text(json.dumps(plan,indent=2))
    jobs=[('candidate',25,candidate),('candidate',40,candidate),('reference',25,{})]
    def run(job):
        name,rate,params=job
        return name+'-'+str(rate),batch(rate,[dict(s,parameters=params) for s in cases],'release_validation/'+name)
    with ThreadPoolExecutor(max_workers=2) as pool:results=dict(pool.map(run,jobs))
    accepted=all(r['passed'] for key,rows in results.items() if key.startswith('candidate') for r in rows)
    (HERE/'release-validation.json').write_text(json.dumps(dict(accepted=accepted,results=results),indent=2))
    print('RELEASE DISTURBANCE GATE: '+str(accepted),flush=True)
if __name__=='__main__':main()
