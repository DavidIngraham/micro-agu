import json,numpy as np,hashlib
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mission2_model import O,PLAN
R=json.loads((O/'synthetic-recovery-results.json').read_text());assert [r['case'] for r in R]==['oracle','sampled','noisy']
fig,axes=plt.subplots(1,3,figsize=(14,4.5))
for ax,k,label in zip(axes,range(3),['10s path RMSE (m)','10s altitude RMSE (m)','10s course RMSE (deg)']):
 for j,name in enumerate(['known','recovered','degraded']):
  ax.bar(np.arange(3)+(j-1)*.25,[r['held_absolute_rmse'][name][k] for r in R],width=.25,label=name)
 ax.set_xticks(range(3),['Exact state','Sampled','Noisy']);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2);ax.legend()
fig.suptitle('Known-model synthetic recovery: same controls, fit objective and held-out scoring');fig.tight_layout();fig.savefig(O/'synthetic-recovery-errors.png',dpi=150)
fig,ax=plt.subplots(figsize=(12,4.5))
for r in R:ax.plot(np.arange(10),np.array(r['relative_parameter_error'])*100,'o-',label=r['case'])
ax.set_xticks(np.arange(10),PLAN['model']['parameters'],rotation=30,ha='right');ax.axhline(0,color='k',lw=.8);ax.set_ylabel('Recovered parameter error (%)');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(O/'synthetic-recovery-parameters.png',dpi=150)
summary=dict(oracle_pass=R[0]['oracle_gate'],all_cases_rank_known_and_recovered_above_degraded=all(all(r['ranking_vs_degraded'].values()) for r in R),maximum_parameter_error_percent={r['case']:float(np.max(abs(np.array(r['relative_parameter_error'])))*100) for r in R},scope='Validates recovery within the coupled mission family under controlled observation assumptions. Does not validate actual parafoil physics, raw BIN decoding, asynchronous sensor timing, hidden-canopy initialization, structural model mismatch, gusts or unobserved rigging changes.',noise='Illustrative independent Gaussian velocity0.15m/s and heading2deg, not a measured sensor model. Prior synthetic excitation is prescribed to make wind observable; actual flight wind identifiability can be worse.',interpretation='The pipeline can recover and rank a correct model; previous real-flight failures are not evidence that the original model is optimal. Sampled initialization introduces parameter bias even when dynamics are correct.',artifacts={name:hashlib.sha256((O/name).read_bytes()).hexdigest() for name in ['synthetic-recovery-plan.json','generate_synthetic_recovery.py','synthetic_recovery_forward.py','run_synthetic_recovery.py','synthetic-recovery-results.json']})
(O/'synthetic-recovery-summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
