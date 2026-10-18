"""Map tracking failure at requested 20/25 levels, without qualifying expansion."""
import json
from run import batch,specs,HERE
def main():
    plan=dict(rates=[20,25],cases=specs()[:2],purpose='15 deg/s failed tracking but stayed bounded with good altitude/speed. Diagnose whether the bias grows at 20/25; no release. Do not proceed above 25 unless all original gates pass there.')
    (HERE/'diagnostic-protocol.json').write_text(json.dumps(plan,indent=2))
    for rate in plan['rates']:
        results=batch(rate,plan['cases'],'diagnostic')
        if not all(r['complete'] for r in results):break
    if rate==25 and all(r['passed'] for r in results):
        for rate in [30,35,40,45,50,55,60]:
            results=batch(rate,plan['cases'],'diagnostic')
            if not all(r['passed'] for r in results):break
if __name__=='__main__':main()
