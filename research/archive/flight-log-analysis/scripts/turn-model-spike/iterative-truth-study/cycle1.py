from pathlib import Path
import json,sys,numpy as np
from scipy.optimize import lsq_linear
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
R=Path(__file__).parent;O=R.parent;sys.path.insert(0,str(O));import up_away_model as M
plan=json.loads((R/'frozen-atlas.json').read_text());rows=plan['rows'];data=np.load(R/'atlas-data.npz');p=np.array(list(json.loads((O/'up-away-A1-fit.json').read_text())['parameters'].values()));ids=[5,6,7,8]; keys=['U','initial','wind','mass','day','voltage','rpm'];basis={};zero={}
# Climb is linear in sink0, climb_thrust, sink_brake and sink_turn at fixed dynamics.
# Build responses through full coupled predictions; these coefficients do not feed speed/course.
for duration in [30,60]:
 indices=[k for k,r in enumerate(rows) if r['duration_s']==duration];D={key:np.concatenate([data[f'{k}_{key}'] for k in indices]) for key in keys};q=p.copy();q[ids]=0;z=M.predict(q,D)
 bs=[]
 for j in ids:
  qq=q.copy();qq[j]=1;bs.append(M.predict(qq,D)[:,:,1]-z[:,:,1])
 for n,k in enumerate(indices):zero[k]=z[n,:,1];basis[k]=np.stack([b[n] for b in bs],axis=1)
X=[];Y=[];W=[];labels=[]
for k,r in enumerate(rows):
 if r['tier']=='ordinary' and r['stratum']!='power-transition':mask=np.ones(len(zero[k]),bool);weight=1/np.sqrt(sum(x['day']==r['day'] and x['tier']=='ordinary' and x['stratum']!='power-transition' for x in rows))/.3
 elif r['tier']=='glide-calibration':
  tt=data[f'{k}_time']+r['start_s'];start=591.24385 if r['day']=='2026-01-03' else 3363.333506;mask=(tt>=start)&(tt<=start+10);weight=1/.1
 else:continue
 X.append(basis[k][mask].mean(axis=0));Y.append((data[f'{k}_observed'][:,1]-zero[k])[mask].mean());W.append(weight);labels.append(k)
X=np.array(X);Y=np.array(Y);W=np.array(W);fit=lsq_linear(X*W[:,None],Y*W,bounds=([.3,.01,0,0],[3,2,3,2]));new=p.copy();new[ids]=fit.x
scores=[];glides=[]
for k,r in enumerate(rows):
 obs=data[f'{k}_observed'][:,1];old=data[f'{k}_baseline'][:,1];c=zero[k]+basis[k]@fit.x;err=c-obs
 scores.append(dict(index=k,day=r['day'],tier=r['tier'],stratum=r['stratum'],old_bias=float((old-obs).mean()),new_bias=float(err.mean()),old_rmse=float(np.sqrt(np.mean((old-obs)**2))),new_rmse=float(np.sqrt(np.mean(err**2)))))
 if r['tier']=='glide-calibration':
  tt=data[f'{k}_time'];start=591.24385 if r['day']=='2026-01-03' else 3363.333506;mask=(tt+r['start_s']>=start)&(tt+r['start_s']<=start+10)
  glides.append(dict(day=r['day'],old_bias=float((old-obs)[mask].mean()),new_bias=float(err[mask].mean())))
  fig,ax=plt.subplots(figsize=(11,4));ax.plot(tt,obs,color='k',label='flight');ax.plot(tt,old,label='A1');ax.plot(tt,c,label='cycle 1');ax.axvspan(start-r['start_s'],start+10-r['start_s'],alpha=.12,color='green',label='calibration interval');ax.set(xlabel='Seconds from continuous context start',ylabel='Climb m/s',title=r['day']+' — glide transition');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(R/f"cycle1-{r['day']}-glide.png",dpi=150);plt.close(fig)
summary={}
for day in ['2026-01-03','2026-02-15']:
 ss=[s for s in scores if s['day']==day and s['tier']=='ordinary' and s['stratum']!='power-transition'];summary[day]={key:float(np.sqrt(np.mean([s[key]**2 for s in ss]))) for key in ['old_bias','new_bias','old_rmse','new_rmse']}
# Leave-one-glide sensitivity: all powered anchors remain; only one glide constraint removed.
transfer=[]
for omitted in [k for k in labels if rows[k]['tier']=='glide-calibration']:
 keep=np.array([k!=omitted for k in labels]);f=lsq_linear(X[keep]*W[keep,None],Y[keep]*W[keep],bounds=([.3,.01,0,0],[3,2,3,2]));j=labels.index(omitted);transfer.append(dict(omitted_day=rows[omitted]['day'],predicted_glide_bias=float(X[j]@f.x-Y[j]),parameters=f.x.tolist()))
result=dict(cycle=1,hypothesis='Glide omission biased sink/thrust tradeoff. Anchor sink with both measured glide events, refit powered thrust response and nonnegative brake/turn sink penalties with speed and steering fixed.',parameters=dict(zip(M.NAMES,new)),baseline_parameters=dict(zip(M.NAMES,p)),fit_success=bool(fit.success),glides=glides,powered_climb=summary,leave_one_glide_out=transfer,decision='Pending plot inspection and reviewer critique; candidate only',course_speed='Exactly unchanged structurally: climb coefficients do not feed V or q in this model.',scores=scores)
(R/'cycle1-result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ['scores','baseline_parameters']},indent=2))
