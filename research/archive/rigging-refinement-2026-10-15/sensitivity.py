from study import *
if __name__=='__main__':
 c=tuple(json.loads((HERE/'selected-final.json').read_text())['routed'][0]['tuning'])
 batch([(f,t,'routed',c,30,a,h,32) for f,(_,starts) in FLIGHTS.items() for t in starts[1::2] for h in [.02,.04,.08] for a in [.25,.45]],'sensitivity-final.json')
