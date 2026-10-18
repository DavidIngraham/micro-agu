from up_away_model import *
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=dict(np.load(O/'up-away-data.npz'));rows=json.loads((O/'up-away-records.json').read_text());P=np.load(O/'up-away-A1-predictions.npz')['prediction'];T=D['truth'];fit=json.loads((O/'up-away-A1-fit.json').read_text());p=np.array([fit['parameters'][k] for k in NAMES])
meas=np.c_[np.trapz(T[:,:,0],dx=.04,axis=1)/10,np.trapz(T[:,:,1],dx=.04,axis=1)/10,np.rad2deg(T[:,-1,2])/10];pred=np.c_[np.trapz(P[:,:,0],dx=.04,axis=1)/10,np.trapz(P[:,:,1],dx=.04,axis=1)/10,np.rad2deg(P[:,-1,2])/10]
fig,ax=plt.subplots(2,3,figsize=(12,7));labels=['Mean ground speed (m/s)','Mean climb (m/s)','Mean course rate (deg/s)']
for d in [0,1]:
 for k in range(3):
  for part,mark in [('development','o'),('check','x')]:
   ix=np.array([r['partition']==part and D['day'][i]==d for i,r in enumerate(rows)]);ax[d,k].scatter(meas[ix,k],pred[ix,k],s=18,marker=mark,label=part,alpha=.65)
  vals=np.r_[meas[D['day']==d,k],pred[D['day']==d,k]];lo,hi=vals.min(),vals.max();ax[d,k].plot([lo,hi],[lo,hi],'k--',lw=1);ax[d,k].set_xlabel('Observed '+labels[k]);ax[d,k].set_ylabel('Predicted');ax[d,k].grid(alpha=.2)
 ax[d,0].set_title(['Hood River','Trout Lake'][d]);ax[d,0].legend()
fig.suptitle('Flight-path model A, fit 1 | frozen development and check episodes');fig.tight_layout();fig.savefig(O/'up-away-A1-checks.png',dpi=160)
# Development-only evidence for deciding whether a second-order response is warranted.
dev=np.array([r['partition']=='development' for r in rows]);e=P[dev,:,2]-T[dev,:,2];lagcorr=[]
for lag in [25,50,100]:lagcorr.append(float(np.corrcoef(e[:,:-lag].ravel(),e[:,lag:].ravel())[0,1]))
report=dict(development_course_residual_lag_correlation_1_2_4s=lagcorr,interpretation='Cumulative-course residual correlation alone cannot establish oscillatory plant dynamics. Do not trigger structure B solely from this statistic.',active_bounds=fit['active_bounds'],check_acceptance=False,remaining_fit_budget={'A':1,'B':2})
(O/'up-away-A1-diagnostic.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
