"""Exploratory follow-up: do not call reused check windows fresh validation."""
import json
import numpy as np
import run
from pathlib import Path
OUT=Path(__file__).resolve().parent
run.CANDIDATES={'baseline':{}}
for yaw in [0,.01,.02,.03]:
 for lag in [0,.15,.4]:
  if yaw==0 and lag==0:continue
  run.CANDIDATES[f'yaw{yaw}_lag{lag}']={'set':{'Cndelta':yaw},'response_tau_s':lag}
train=[(f,t) for f,(_,starts) in run.study.FLIGHTS.items() for t in starts[::2]]+[('hood_river',438)]
def main():
 (OUT/'lag-protocol.json').write_text(json.dumps({'candidates':run.CANDIDATES,'purpose':'Explore a combined steering-authority and brake-response hypothesis after examining the first screen; no fresh validation or promotion claim.'},indent=2)+'\n')
 rows=run.batch([(f,t,'lag-screen',name,.35) for name in run.CANDIDATES for f,t in train]);(OUT/'lag-training.json').write_text(json.dumps(rows,indent=2)+'\n')
 base={(r['flight'],r['start']):r for r in rows if r['candidate']=='baseline'}
 ranks=[]
 for name in run.CANDIDATES:
  group=[r for r in rows if r['candidate']==name]
  score=np.mean([np.mean([r['metrics']['settled'][k]/base[r['flight'],r['start']]['metrics']['settled'][k] for k in run.METRICS]) for r in group])
  turn=next(r for r in group if r['start']==438)
  ranks.append({'candidate':name,'mean_score':float(score),'turn':turn['turn_summary']['model']})
 ranks.sort(key=lambda r:r['mean_score']);(OUT/'lag-ranking.json').write_text(json.dumps(ranks,indent=2)+'\n');print(json.dumps(ranks,indent=2))
if __name__=='__main__':main()
