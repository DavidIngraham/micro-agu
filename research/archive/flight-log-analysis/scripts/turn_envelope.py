from pathlib import Path
import numpy as np,json
from scipy.optimize import least_squares
from scipy.ndimage import uniform_filter1d
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/longitudinal-2026-10-09/turn-model-spike')
results={};fig,ax=plt.subplots(2,2,figsize=(12,8),layout='constrained')
for col,day in enumerate(['2026-01-03','2026-02-15']):
 z=dict(np.load(O/(day+'.npz')));t=z['t'];rows=[]
 for lo in np.arange(np.ceil(t[0]),t[-1]-10,5):
  m=(t>=lo)&(t<lo+10)
  if not np.all(z['airborne'][m]) or not np.all(np.isin(z['mode'][m],[0,5,7,10,12,15])):continue
  # Sign consistency and minimum net rotation identify sustained turning windows.
  angle=z['yaw'][m][-1]-z['yaw'][m][0];rate=angle/10
  if abs(angle)<30 or np.mean(np.sign(rate)*z['heading_rate'][m]>0)<.85:continue
  rows.append(dict(start=float(lo),bank=float(np.median(abs(z['roll'][m]))),rate=abs(float(rate)),throttle=float(np.mean(z['throttle_pwm'][m])),climb=float(np.mean(z['climb'][m])),brake=float(np.mean(abs(z['differential'][m]))),common=float(np.mean(z['common'][m])),speed=float(np.mean(z['spd'][m])),mode=int(np.median(z['mode'][m]))))
 ax[0,col].scatter([x['bank'] for x in rows],[x['rate'] for x in rows],c=[x['throttle'] for x in rows],vmin=0,vmax=.7,cmap='viridis',s=16);ax[0,col].set_xlabel('Median absolute payload bank (deg)');ax[0,col].set_ylabel('Net heading rate (deg/s)');ax[0,col].set_title(day+' · 10 s turning windows')
 sc=ax[1,col].scatter([x['rate'] for x in rows],[x['climb'] for x in rows],c=[x['throttle'] for x in rows],vmin=0,vmax=.7,cmap='viridis',s=16);ax[1,col].set_xlabel('Net heading rate (deg/s)');ax[1,col].set_ylabel('Mean climb (m/s)');ax[1,col].axhline(0,color='grey',lw=.7)
 for a in ax[:,col]:a.grid(alpha=.2)
 # Estimate constant wind from velocity-circle fit over selected turns. Assumes constant airspeed, not a sensor measurement.
 fits=[]
 for lo,hi in ([(447.16,452.20),(508.16,515.08)] if col==0 else [(780,825),(950,1045),(3387.37,3392.77)]):
  m=(t>=lo)&(t<=hi);vn=z['VN'][m];ve=z['VE'][m]
  sol=least_squares(lambda a:np.hypot(vn-a[0],ve-a[1])-a[2],[np.mean(vn),np.mean(ve),5],bounds=([-10,-10,.1],[10,10,20]),loss='soft_l1');wn,we,v=sol.x;res=np.sqrt(np.mean(sol.fun**2));yawspan=abs(z['yaw'][m][-1]-z['yaw'][m][0]);radius=v/(np.deg2rad(yawspan)/(hi-lo))
  fits.append(dict(start=lo,end=hi,wind_N=wn,wind_E=we,airspeed_assumed_constant=v,speed_residual_rms=res,radius_heading_proxy=radius))
 results[day]=dict(turn_windows=rows,wind_circle_fits=fits)
 print(day,'windows',len(rows),'wind fits',fits)
fig.colorbar(sc,ax=ax.ravel().tolist(),label='Throttle output fraction');fig.suptitle('Measured turn envelope: overlapping 10 s windows, no claim of steady equilibrium');fig.savefig(O/'measured-turn-envelope.png',dpi=150)
(O/'turn-envelope.json').write_text(json.dumps(results,indent=2))
