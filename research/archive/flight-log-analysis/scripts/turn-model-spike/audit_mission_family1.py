from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from fit_mission_family1 import dataset,predict,names,rows
O=Path(__file__).parent
D=dataset(rows);fit=json.loads((O/'mission-family1-development-fit.json').read_text());p=[fit['parameters'][n] for n in names];pred=predict(p,D);truth=np.array([d['truth'] for d in D]);report=[]
fig,axes=plt.subplots(2,2,figsize=(11,8))
for j,day in enumerate(['2026-01-03','2026-02-15']):
 ii=np.array([d['row']['day']==day for d in D]);dd=[d for d in D if d['row']['day']==day]
 diff=np.array([np.mean(d['u'][:,2]-d['u'][:,1])*72 for d in dd]);turn=truth[ii,2];initial=np.rad2deg([d['init'][3] for d in dd]);res=pred[ii,2]-turn
 X=np.c_[np.ones(len(dd)),[d['u'][:,1].mean() for d in dd],[d['u'][:,2].mean() for d in dd]];coef=np.linalg.lstsq(X,turn,rcond=None)[0]
 report.append(dict(day=day,n=len(dd),right_minus_left_vs_ground_turn_correlation=float(np.corrcoef(diff,turn)[0,1]),descriptive_ols_intercept_left_right_deg_s=coef.tolist(),initial_turn_vs_turn_error_correlation=float(np.corrcoef(initial,res)[0,1]),median_abs_initial_air_course_rate_deg_s=float(np.median(abs(initial))),development_turn_rmse_deg_s=float(np.sqrt(np.mean(res**2))),caution='Closed-loop associations, not causal actuator identification; no model refit.'))
 axes[j,0].scatter(diff,turn);axes[j,0].set(xlabel='Mean right minus left pull (mm)',ylabel='Observed net ground turn (deg/s)',title=day+' development controls')
 axes[j,1].scatter(initial,res);axes[j,1].set(xlabel='Initial air-course rate (deg/s)',ylabel='Turn prediction error (deg/s)',title='Initialization sensitivity diagnostic')
 for ax in axes[j]:ax.axhline(0,color='gray',lw=.7);ax.grid(alpha=.2)
result=dict(scope='Development-only descriptive audit; no coefficient fitting or evaluation-based selection',calibration='Source audit_causal_resampling.py: left=(C3-800)*pi*55/360*0.1 mm; right=(2400-C4)*same; U columns throttle,left/72,right/72. Mission model consumes this order correctly. Physical left/right labeling comes from existing calibration, not independently verified here.',findings=report)
(O/'mission-family1-development-audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2));fig.tight_layout();fig.savefig(O/'mission-family1-development-audit.png',dpi=150)
