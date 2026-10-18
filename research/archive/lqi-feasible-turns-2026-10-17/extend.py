"""Extend the successful anti-windup-only ablation, fixed gains and observer."""
import json
from run import batch,specs,HERE
def main():
    plan=dict(rates=[30,35,40,45,50],parameters={'CP3_AWMODE':1},cases=specs()[:2],
        stop='First original gate failure. Do not test 60: local linear analysis already identifies unstable nodes there. Exploratory nominal envelope only.')
    (HERE/'extension-protocol.json').write_text(json.dumps(plan,indent=2))
    for rate in plan['rates']:
        results=batch(rate,[dict(s,parameters=plan['parameters']) for s in plan['cases']],'extension/aw')
        if not all(r['passed'] for r in results):break
if __name__=='__main__':main()
