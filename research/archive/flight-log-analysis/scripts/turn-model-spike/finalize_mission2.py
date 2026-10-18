import json,hashlib,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mission2_model import O,PLAN
C=json.loads((O/'mission2-comparison.json').read_text());E=json.loads((O/'mission2-candidate-evaluation.json').read_text());rr=E['records'];truth=np.array(E['observed']);models={'Coupled':np.array(E['candidate']),'Constant':np.array(E['constant_reference']),'Detailed':np.array(C['detailed_predictions'],dtype=float)}
fig,axes=plt.subplots(2,3,figsize=(13,7))
for j,day in enumerate(['2026-01-03','2026-02-15']):
 ii=np.array([r['day']==day for r in rr]);x=np.arange(ii.sum());labels=[str(round(r['start_s'])) for r in rr if r['day']==day]
 for k,label in enumerate(['Path error (m)','Altitude-change error (m)','Course-change error (deg)']):
  ax=axes[j,k]
  for name,p in models.items():ax.plot(x,(p-truth)[ii,k],marker='o',label=name)
  ax.axhline(0,color='k',lw=.8);ax.set_xticks(x,labels);ax.set(title=day+' '+label,xlabel='Window start (s)');ax.grid(alpha=.2)
  if k==0:ax.legend()
fig.suptitle('Frozen evaluation: gaps indicate divergent detailed-baseline forecasts');fig.tight_layout();fig.savefig(O/'mission2-final-comparison.png',dpi=150)
manifest=json.loads((O/'baseline-before-napkin/manifest.json').read_text());hashchecks={name:hashlib.sha256((O/'baseline-before-napkin'/name).read_bytes()).hexdigest()==sha for name,sha in manifest.items()};assert all(hashchecks.values())
raw=json.loads((O/'mission2-detailed-results.json').read_text());assert len(raw)==36;assert len(rr)==12
failures=[x for x in C['selected_starts'] if x['status']!='finite']
sens=[];scale=np.array(PLAN['normalization_development_std'])
for s in E['wind_sensitivity']:
 pred=np.array(s['predictions']);scores={}
 for day in ['2026-01-03','2026-02-15']:
  vals=[]
  for cat in {r['category'] for r in rr if r['day']==day}:
   ii=np.array([r['day']==day and r['category']==cat for r in rr]);vals.append(np.mean(((pred[ii]-truth[ii])/scale)**2))
  scores[day]=float(np.mean(vals))
 sens.append(dict(wind_delta=s['wind_delta_N_E_m_s'],scores=scores,all_finite=s['all_finite']))
report=dict(decision=C['decision'],baseline_hashes_unchanged=hashchecks,fit_attempts=1,detailed_trials=len(raw),evaluation_windows=len(rr),selected_baseline_failures=failures,wind_sensitivity=sens,remaining_limitations=['Only one glide window per flight; glide performance not independently validated','Wind from payload heading relies on negligible mean sideslip; +/-0.5m/s is a sensitivity probe, not measured uncertainty','Velocity and course share EKF sources; no independent measurement-error covariance was available, so measurement-precision claims are unsupported','Observed-to-predicted starting state differs across reduced/detailed models; disclosed in comparison report','Induced-drag coefficient at lower bound undermines physical identification of sustained turning losses','No maximum sustained turn radius or maximum climb envelope inferred from these short controlled-response forecasts'],completion_scope='Bounded experiment completed with rejection; previous net-improvement objective remains unmet')
(O/'mission2-final-audit.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(failures=failures,wind_sensitivity=sens,baseline_preserved=all(hashchecks.values())),indent=2))
