from up_away_model import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
models=['A1','A2','B1','B2'];fig,ax=plt.subplots(1,3,figsize=(13,4));labels=[];values=[]
for day in ['0','1']:
 for cat in ['glide','straight','left','right']:
  labels.append(('HR' if day=='0' else 'TL')+' '+cat);values.append([json.loads((O/f'up-away-{m}-checks.json').read_text())[day][cat]['rmse_speed_climb_turn'] for m in models])
a=np.array(values)
for k,(title,budget) in enumerate([('Speed RMSE m/s',.6),('Climb RMSE m/s',.3),('Course-rate RMSE deg/s',2)]):
 for j,m in enumerate(models):ax[k].plot(range(8),a[:,j,k],marker='o',ms=3,label=m)
 ax[k].axhline(budget,color='k',ls='--',label='frozen budget');ax[k].set_xticks(range(8),labels,rotation=50,ha='right');ax[k].set_title(title);ax[k].grid(alpha=.2)
ax[0].legend(fontsize=8);fig.suptitle('Four bounded fits: no candidate passes the complete acceptance matrix');fig.tight_layout();fig.savefig(O/'up-away-four-fit-comparison.png',dpi=170)
report=dict(status='Fitting budget exhausted; simulator not accepted; packaging and controller/numerical diagnostics remain',fits=[dict(model=m,fit=json.loads((O/f'up-away-{m}-fit.json').read_text())) for m in models],unresolved=['Glide sink error approximately0.4-0.56m/s, above0.3m/s budget','Trout Lake turn-rate errors approximately4-6.4deg/s powered and12deg/s glide, above2deg/s budget','Control-response gain/lag and coupling validation incomplete','No calibrated model-discrepancy ensemble or completed controller-in-loop acceptance yet'],next='Preserve and package runnable candidates; verify long-horizon numerics and exercise controllers diagnostically, without calling them validated. Identify minimum data to distinguish trim/authority from throttle coupling and support low-power response.')
(O/'up-away-bounded-fit-decision.json').write_text(json.dumps(report,indent=2))
