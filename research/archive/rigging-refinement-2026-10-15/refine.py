from study import *
if __name__=='__main__':
 fit=[(f,t) for f,(_,starts) in FLIGHTS.items() for t in starts[::2]]
 candidates=list(itertools.product([1.8,2.5],[.85,1.,1.15],[1.,.0743/.066]))
 rows=batch([(f,t,v,c,10,.35,.06,32) for v in ['legacy','routed'] for c in candidates for f,t in fit],'fit-extension.json')
 rows+=json.loads((HERE/'fit.json').read_text())
 baseline={(r['flight'],r['start']):r['metrics'] for r in rows if r['variant']=='legacy' and r['tuning']==[1.,1.,1.]}
 def score(group):
  if not all(r['finite'] for r in group):return 1e6
  return float(np.mean([sum(w*r['metrics'][k]/max(baseline[r['flight'],r['start']][k],1e-6) for k,w in [('roll_rate',1),('pitch_rate',1),('yaw_rate',1),('bank',.25),('climb',.25)])/3.5 for r in group]))
 ranked={v:sorted([dict(tuning=list(c),score=score([r for r in rows if r['variant']==v and r['tuning']==list(c)])) for c in sorted(set(tuple(r['tuning']) for r in rows))],key=lambda r:r['score']) for v in ['legacy','routed']}
 (HERE/'selected-final.json').write_text(json.dumps(ranked,indent=2))
 batch([(f,t,v,tuple(ranked[v][0]['tuning']),30,a,.06,32) for f,(_,starts) in FLIGHTS.items() for t in starts for a in [.25,.35,.45] for v in ranked],'evaluation-final.json')
 print({k:v[0] for k,v in ranked.items()})
