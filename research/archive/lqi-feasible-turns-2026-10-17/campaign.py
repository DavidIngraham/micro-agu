"""Frozen ablations before selecting an envelope candidate."""
from concurrent.futures import ThreadPoolExecutor
import json
from run import batch,specs,HERE
VARIANTS={'reference':{},'aw':{'CP3_AWMODE':1},'map':{'CP3_TMAP':1},'map_aw':{'CP3_TMAP':1,'CP3_AWMODE':1},'map_aw_observer':{'CP3_TMAP':1,'CP3_AWMODE':1,'CP3_OBSTRIM':1}}
def main():
    protocol=dict(variants=VARIANTS,rate=25,cases=specs()[:2],gates='Unchanged prior turn-envelope score.py; all nominal cases must pass. Select simplest passing intervention; assess observer error and local poles separately. No gain retuning.')
    (HERE/'protocol.json').write_text(json.dumps(protocol,indent=2))
    def run(item):
        name,params=item;ss=[dict(s,parameters=params) for s in protocol['cases']]
        return name,batch(25,ss,'ablation/'+name)
    with ThreadPoolExecutor(max_workers=2) as pool:results=dict(pool.map(run,VARIANTS.items()))
    (HERE/'ablation.json').write_text(json.dumps(results,indent=2))
if __name__=='__main__':main()
