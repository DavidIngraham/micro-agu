"""Nonlinear trim/observer equilibrium scheduling with fixed K and L."""
import json
from run import batch,specs,HERE
def main():
    plan=dict(rates=[10,15,20,30,35,40],parameters={'CP3_AWMODE':1,'CP3_TMAP':1,'CP3_OBSTRIM':1},cases=specs()[:2],
        stop='First original gate failure. Table currently supports up to 40 deg/s; no extrapolation. No feedback/observer gain scheduling.')
    (HERE/'map-extension-protocol.json').write_text(json.dumps(plan,indent=2))
    for rate in plan['rates']:
        results=batch(rate,[dict(s,parameters=plan['parameters']) for s in plan['cases']],'extension/map_aw_observer')
        if not all(r['passed'] for r in results):break
if __name__=='__main__':main()
