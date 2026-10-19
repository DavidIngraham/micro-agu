import json,hashlib
import numpy as np
from pathlib import Path
from scipy.signal import welch
OUT=Path(__file__).resolve().parent
BASE=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09')
rows=[];hashes={}
for folder,starts in [('2026-01-03 15-41-42',[580,594,1060]),('2026-02-15 15-38-50',[3350,3366,3373,3443,3459])]:
 def read(k):
  path=BASE/folder/(k+'.json');hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
  r=json.loads(path.read_text());r=[x for x in r if k!='IMU' or x['I']==0]
  return np.array([x['TimeUS']/1e6 for x in r]),r
 ti,im=read('IMU');tp,p=read('POS');te,e=read('ESC');tr,r=read('RCOU')
 for start in starts:
  q=np.arange(start,start+6,.02)
  assert max(np.diff(ti[(ti>=start)&(ti<=start+6)]))<.06
  g=np.column_stack([np.interp(q,ti,[x[k] for x in im]) for k in ['GyrX','GyrY','GyrZ']])*180/np.pi
  f,P=welch(g,fs=50,nperseg=300,axis=0);band=(f>=1.2)&(f<=1.9);wide=(f>=.5)&(f<=5)
  ei=np.maximum(0,np.searchsorted(te,q,side='right')-1)
  rpm=np.array([x.get('RawRPM',x['RPM']) for x in e])[ei]
  th=np.array([x['C1'] for x in r])[np.maximum(0,np.searchsorted(tr,q,side='right')-1)]
  ri=np.maximum(0,np.searchsorted(tr,q,side='right')-1)
  controls=np.array([[x['C3'],x['C4']] for x in r])[ri]
  left=(controls[:,0]-800)*np.pi*55/360*.1;right=(2400-controls[:,1])*np.pi*55/360*.1
  cf,cp=welch(np.column_stack([right-left,(right+left)/2]),fs=50,nperseg=300,axis=0)
  alt=np.interp(q,tp,[x['RelHomeAlt'] for x in p])
  rows.append(dict(flight=folder,start=start,end=start+6,altitude=alt[[0,-1]].tolist(),rpm_min_median_max=np.quantile(rpm,[0,.5,1]).tolist(),throttle_pwm_min_median_max=np.quantile(th,[0,.5,1]).tolist(),max_esc_age=float(max(q-te[ei])),band_rms=np.sqrt(P[band].sum(0)*(f[1]-f[0])).tolist(),peak_hz=f[wide][np.argmax(P[wide],axis=0)].tolist(),frequency=f.tolist(),rate_psd=P.tolist(),brake_psd=cp.tolist(),brake_peak_hz=f[wide][np.argmax(cp[wide],axis=0)].tolist()))
(OUT/'flight-windows.json').write_text(json.dumps(rows,indent=2))
(OUT/'input-hashes.json').write_text(json.dumps(hashes,indent=2))
