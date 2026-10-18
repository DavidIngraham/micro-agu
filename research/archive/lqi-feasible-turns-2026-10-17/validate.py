"""Fresh paired disturbance checks; preserve absolute screens and compare metrics."""
from concurrent.futures import ThreadPoolExecutor
import json
from run import batch,specs,HERE
def main():
    variants={'reference':{},'aw':{'CP3_AWMODE':1},'map_aw':{'CP3_AWMODE':1,'CP3_TMAP':1},'map_aw_observer':{'CP3_AWMODE':1,'CP3_TMAP':1,'CP3_OBSTRIM':1}}
    plan=dict(rate=25,variants=variants,cases=specs(True)[:6],
        interpretation='Fresh cases never used to modify gains/trim/anti-windup. Retain previous absolute gates, including known turbulence failures. Compare sustained-turn bias, altitude, motion, speed, saturation, recovery and shaft power. No broad qualification from relative improvements alone.')
    (HERE/'validation-protocol.json').write_text(json.dumps(plan,indent=2))
    def run(item):
        name,params=item;return name,batch(25,[dict(s,parameters=params) for s in plan['cases']],'validation/'+name)
    with ThreadPoolExecutor(max_workers=2) as pool:results=dict(pool.map(run,variants.items()))
    (HERE/'validation.json').write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
