from up_away_model import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=dict(np.load(O/'up-away-data.npz'));rows=json.loads((O/'up-away-records.json').read_text());p=np.array([json.loads((O/'up-away-A1-fit.json').read_text())['parameters'][k] for k in NAMES]);P=np.load(O/'up-away-A1-predictions.npz')['prediction'];T=D['truth'];dev=np.array([r['partition']=='development' for r in rows]);tt=np.arange(251)*.04
metric=lambda x:np.c_[np.trapz(x[:,:,0],dx=.04,axis=1)/10,np.trapz(x[:,:,1],dx=.04,axis=1)/10,np.rad2deg(x[:,-1,2])/10]
y=metric(T);pred=metric(P);persist=np.c_[T[:,0,0],T[:,0,1],np.rad2deg(D['initial'][:,3])];constant=np.zeros_like(y)
for d in [0,1]:constant[D['day']==d]=y[dev&(D['day']==d)].mean(axis=0)
summary={}
for part,mask in [('development',dev),('check',~dev)]:
 summary[part]={}
 for d in [0,1]:
  ix=mask&(D['day']==d);summary[part][str(d)]={name:np.sqrt(np.mean((z[ix]-y[ix])**2,axis=0)).tolist() for name,z in [('A1',pred),('persistence_air_rate_proxy',persist),('development_constant',constant)]}
# Investigate development temporal response without adjusting any coefficients.
fig,ax=plt.subplots(4,3,figsize=(14,11));selected=[]
for d in [0,1]:
 ids=np.flatnonzero(dev&(D['day']==d));order=ids[np.argsort(abs(pred[ids,2]-y[ids,2]))];selected.extend([int(order[len(order)//2]),int(order[-1])])
for j,i in enumerate(selected):
 u=D['U'][i];ax[j,0].plot(tt,u[:,1]*72,label='left');ax[j,0].plot(tt,u[:,2]*72,label='right');ax[j,0].set_ylabel('Brake mm');ax[j,0].legend();ax[j,0].set_title(rows[i]['day']+' @ '+str(round(rows[i]['start_s'],1))+'s')
 ax[j,1].plot(tt,np.rad2deg(T[i,:,2]),label='observed ground course');ax[j,1].plot(tt,np.rad2deg(P[i,:,2]),label='A1');ax[j,1].set_ylabel('Course change deg');ax[j,1].legend()
 ax[j,2].plot(tt,T[i,:,1],label='observed climb');ax[j,2].plot(tt,P[i,:,1],label='A1');ax[j,2].set_ylabel('Climb m/s');ax[j,2].legend()
 for a in ax[j]:a.set_xlabel('Forecast seconds');a.grid(alpha=.2)
fig.suptitle('Development only: median and worst turn-error episode per flight');fig.tight_layout();fig.savefig(O/'up-away-A1-development-traces.png',dpi=150)
# Correlation of mean forcing columns estimates confounding, not causality.
aT=thrust(rpm_target(D['U'][:,:,0],D['voltage'][:,None]),np.maximum(P[:,:,3],1))/D['mass'][:,None]
x=np.c_[D['U'][:,:,1].mean(axis=1),D['U'][:,:,2].mean(axis=1),aT.mean(axis=1)]
conf={str(d):np.corrcoef(x[dev&(D['day']==d)].T).tolist() for d in [0,1]}
# Pure numerical refinement; coefficients unchanged.
fine=predict(p,D,substeps=4);delta=np.max(abs(metric(fine)-pred),axis=0)
result=dict(errors_order=['speed_m_s','climb_m_s','course_rate_deg_s'],comparisons=summary,development_mean_forcing_correlation_left_right_thrust=conf,timestep_refinement_max_delta=delta.tolist(),plot_selected_development_indices=selected,interpretation='Persistence turn entry is air-rate proxy, not exact ground-rate persistence. Time traces chosen only from development. This audit does not refit the model.')
(O/'up-away-A1-response-audit.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
