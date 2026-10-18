"""Reconstruct the powered Hood River 360+ turn from causal logged samples."""
from common import *
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
plt.rcParams.update({'figure.dpi':140,'axes.grid':True,'grid.alpha':.25})
a=np.load(SOURCE/'2026-01-03-causal-input.npz');old=np.load(SOURCE/'2026-01-03.npz');t=a['t'];y=a['Y'];dt=np.median(np.diff(t))
params={}
for folder in ['2026-01-03 15-41-42','2026-02-15 15-38-50']:
 raw=json.loads((SOURCE.parent/folder/'PARM.json').read_text())
 params[folder]=[x for x in raw if x.get('Name') in [f'SERVO{s}_{k}' for s in [1,3,4] for k in ['MIN','MAX','TRIM','REVERSED','FUNCTION']]]
(OUT/'logged-servo-parameters.json').write_text(json.dumps(params,indent=2))
# Reconstruct independently from original output PWM and the logged zero positions.
r=json.loads((SOURCE.parent/'2026-01-03 15-41-42/RCOU.json').read_text());rt=np.array([x['TimeUS']*1e-6 for x in r]);idx=np.maximum(0,np.searchsorted(rt,t,side='right')-1)
pwm=np.array([[x['C1'],x['C3'],x['C4']] for x in r])[idx]
reconstructed=np.c_[pwm[:,1]-800,2400-pwm[:,2]]*np.pi*55/360*.1
m=(t>=447)&(t<=456);plot=(t>=439)&(t<=463)
assert a['valid'][m].all()
assert np.max(abs(reconstructed[m]-a['brake_mm'][m]))<1e-8
v=y[:,6:9];course=np.unwrap(np.arctan2(v[:,1],v[:,0]));smooth=savgol_filter(v,25,3,axis=0);acc=savgol_filter(v,25,3,deriv=1,delta=dt,axis=0)
rate=(smooth[:,0]*acc[:,1]-smooth[:,1]*acc[:,0])/np.maximum(np.sum(smooth[:,:2]**2,axis=1),.01)
load=np.linalg.norm(a['force'],axis=1)/9.80665;wind=a['wind'];air=np.linalg.norm(v-wind,axis=1)
metrics=dict(window_boot_s=[447,456],payload_peak_bank_deg=float(np.rad2deg(y[m,0]).max()),payload_mean_bank_deg=float(np.rad2deg(y[m,0]).mean()),yaw_change_deg=float(np.rad2deg(y[m,2][-1]-y[m,2][0])),ground_course_change_deg=float(np.rad2deg(course[m][-1]-course[m][0])),mean_groundspeed=float(np.linalg.norm(v[m,:2],axis=1).mean()),mean_sink=float(v[m,2].mean()),mean_throttle=float(a['U'][m,0].mean()),throttle_range=a['U'][m,0][[np.argmin(a['U'][m,0]),np.argmax(a['U'][m,0])]].tolist(),max_brakes_mm=a['brake_mm'][m].max(0).tolist(),mean_brakes_mm=a['brake_mm'][m].mean(0).tolist(),mean_specific_load_g=float(load[m].mean()),p95_specific_load_g=float(np.percentile(load[m],95)),inferred_mean_airspeed=float(air[m].mean()),mean_assumed_wind=wind[m].mean(0).tolist(),pwm_reconstruction_max_error_mm=float(np.max(abs(reconstructed[m]-a['brake_mm'][m]))))
(OUT/'hood-river.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
fig,ax=plt.subplots(4,2,figsize=(13,13),layout='constrained');ax=ax.ravel()
ax[0].plot(old['PE'][plot]-old['PE'][plot][0],old['PN'][plot]-old['PN'][plot][0],color='0.7');sc=ax[0].scatter(old['PE'][m]-old['PE'][plot][0],old['PN'][m]-old['PN'][plot][0],c=t[m],s=7,cmap='viridis');fig.colorbar(sc,ax=ax[0],label='Boot time (s)');ax[0].set(xlabel='East (m)',ylabel='North (m)',title='Logged ground track');ax[0].set_aspect('equal')
ax[1].plot(t[plot],np.rad2deg(y[plot,0]),label='Payload bank');ax[1].set(ylabel='Degrees',title='Payload attitude, not canopy attitude')
ax[2].plot(t[plot],np.rad2deg(course[plot]-course[plot][0]),label='Ground course');ax[2].plot(t[plot],np.rad2deg(y[plot,2]-y[plot,2][0]),label='Payload yaw');ax[2].set(ylabel='Degrees',title='A complete turn and more')
ax[3].plot(t[plot],np.linalg.norm(v[plot,:2],axis=1),label='Measured ground speed');ax[3].plot(t[plot],air[plot],label='Wind-estimated airspeed',ls='--');ax[3].set(ylabel='m/s',title='Airspeed is inferred, not measured')
ax[4].plot(t[plot],a['brake_mm'][plot,0],label='Left');ax[4].plot(t[plot],a['brake_mm'][plot,1],label='Right');ax[4].axhline(62.3955,ls=':',c='C0');ax[4].axhline(71.9948,ls=':',c='C1');ax[4].set(ylabel='Commanded pull (mm)',title='Logged PWM converted using 55 mm drums')
ax[5].plot(t[plot],100*a['U'][plot,0],label='Throttle output');ax[5].set(ylabel='Percent',title='Powered throughout the highlighted turn')
ax[6].plot(t[plot],v[plot,2],label='Down velocity');ax[6].axhline(0,c='k',lw=.7);ax[6].set(ylabel='Sink (m/s)',title='Positive down: this turn gains height on average')
ax[7].plot(t[plot],load[plot],alpha=.4,label='IMU specific-force magnitude');ax[7].plot(t[plot],savgol_filter(load,25,3)[plot],label='1 s smoothing');ax[7].set(ylabel='g',title='IMU loading includes payload dynamics')
for x in ax[1:]:x.axvspan(447,456,color='green',alpha=.08);x.set_xlabel('Boot time (s)');x.legend(fontsize=8)
fig.suptitle('Hood River: powered 360+ turn, 3 January 2026',fontsize=17);fig.savefig(OUT/'hood-river.png');plt.close(fig)
