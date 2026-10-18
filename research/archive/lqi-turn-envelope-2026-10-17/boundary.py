"""Refine the first failed calm-air level; compare classical behavior."""
import json
from run import batch,specs,HERE
def main():
    plan=dict(rates=[12,13,14],cases=specs()[:2],classical_rates=[15,25],
        reason='15 deg/s nominal tracking failed before instability; refine below failure without relaxing qualification gates. No release.')
    (HERE/'boundary-protocol.json').write_text(json.dumps(plan,indent=2))
    for rate in plan['rates']:
        if not all(r['passed'] for r in batch(rate,plan['cases'],'nominal')):break
    for rate in plan['classical_rates']:batch(rate,specs()[:4],'comparison','baseline')
if __name__=='__main__':main()
