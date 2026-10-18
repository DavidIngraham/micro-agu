"""Separate calm-air exploration after the existing limit failed turbulent gates.

Does not relax the original broad-envelope acceptance gate or authorize release.
Frozen before any >10 deg/s result. Stop at the first nominal failed level.
"""
import json
from run import batch,specs,HERE
def main():
    plan=dict(rates=list(range(15,61,5)),cases=specs()[:2],purpose='Calm-air achievable envelope only; no broad-envelope promotion; unchanged score.py gates')
    (HERE/'nominal-protocol.json').write_text(json.dumps(plan,indent=2))
    for rate in plan['rates']:
        results=batch(rate,plan['cases'],'nominal')
        if not all(r['passed'] for r in results):
            print('NOMINAL STOP AT '+str(rate),flush=True);break
if __name__=='__main__':main()
