import copy,json
import numpy as np
from scipy.signal import welch
from run import OUT,segments,variants,one
rows=[]
for s in segments():
 for k in ['k1.0-c1.0-j1.0','k1.0-c0.05-j1.3']:
  for kind in ['recorded32','recorded64','constant_brakes']:
   ss=copy.copy(s);o=variants()[k].copy()
   if kind=='recorded64':o['substeps']=64
   if kind=='constant_brakes':
    ss.commands=s.commands.copy();ss.commands[:,1:]=np.mean(ss.commands[:,1:],axis=0)
   r=one((ss,k,o,[0,0,0,0],True));r['kind']=kind;rows.append(r)
for s in segments():
 for k in ['k1.0-c1.0-j1.0','k1.0-c0.05-j1.3']:
  a,b=[r for r in rows if r['name']==s.name and r['variant']==k and r['kind'] in ['recorded32','recorded64']]
  assert max(abs(np.array(a['trace'])-b['trace']).ravel())<.02
(OUT/'checks.json').write_text(json.dumps(rows,separators=(',',':')))
for r in rows:
 if r['name']=='trout_lake-3366' and r['variant']=='k1.0-c0.05-j1.3':
  g=np.array(r['trace']);f,P=welch(g[200:-1],fs=25,nperseg=150,axis=0);b=(f>=1.8)&(f<=2.4)
  print(r['kind'],'full band',r['model']['band_rms'][2],'late band',np.sqrt(P[b,2].sum()*(f[1]-f[0])))
