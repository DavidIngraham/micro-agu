from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;rows=[];summaries=[]
fig,axs=plt.subplots(2,2,figsize=(12,7))
for col,day in enumerate(['2026-01-03','2026-02-15']):
 a=np.load(O/f'{day}-causal-input.npz');b=np.load(O/f'{day}.npz');t=a['t'];y=a['Y'];air=np.interp(t,b['t'],b['airborne'].astype(float))>.99
 valid=air&a['valid'].astype(bool)&np.all(np.isfinite(y),axis=1);bank=np.rad2deg(y[:,0]);speed=np.linalg.norm(y[:,6:8],axis=1);climb=-y[:,8];thr=a['U'][:,0]
 axs[0,col].hist(abs(bank[valid]),bins=np.arange(0,92,2),weights=np.ones(valid.sum())*.04);axs[0,col].axvline(30,color='k',linestyle='--',label='Exploratory 30° boundary');axs[0,col].set(xlabel='Absolute payload bank (deg)',ylabel='Logged time (s)',title='Hood River' if col==0 else 'Trout Lake');axs[0,col].legend(fontsize=8)
 axs[1,col].scatter(thr[valid][::10],climb[valid][::10],c=np.minimum(abs(bank[valid][::10]),60),s=4,cmap='viridis',vmin=0,vmax=60);axs[1,col].set(xlabel='Throttle command (fraction)',ylabel='Climb rate (m/s)')
 counts={}
 # Fixed chronological, non-overlapping 15-second grid: selection never reads predictions.
 for start in range(int(np.ceil(t[0]/15)*15),int(t[-1])-15,15):
  ix=np.searchsorted(t,start);sl=slice(ix,ix+376)
  if ix+376>len(t) or not np.all(valid[sl]) or np.min(speed[sl])<2:continue
  bk=bank[sl];th=thr[sl];v=climb[sl];course=np.unwrap(np.arctan2(y[sl,7],y[sl,6]));turn=np.rad2deg(course[-1]-course[0]);mx=float(np.max(abs(bk)));med=float(np.median(abs(bk)))
  rate95=float(np.percentile(abs(np.rad2deg(course[25:]-course[:-25])),95))
  if mx>30 or rate95>15:category='high_bank_or_fast_turn_stress'
  elif abs(turn)>=30 and med>=5:category='moderate_right' if turn>0 else 'moderate_left'
  elif np.max(th)<=.08:category='low_power_glide'
  elif np.median(v)>=.3 and np.ptp(th)<=.08:category='steady_powered_climb'
  elif np.ptp(th)>=.15:category='throttle_transition'
  else:category='other_mission_motion'
  counts[category]=counts.get(category,0)+1
  rows.append(dict(day=day,start_s=start,cutoff_s=start+5,category=category,course_rate_p95_deg_s=rate95,max_abs_bank_deg=mx,median_abs_bank_deg=med,course_change_deg=float(turn),mean_speed_m_s=float(np.mean(speed[sl])),mean_climb_m_s=float(np.mean(v)),mean_throttle=float(np.mean(th)),throttle_range=float(np.ptp(th))))
 summaries.append(dict(day=day,valid_airborne_s=float(valid.sum()*.04),fraction_abs_bank_at_most_30=float(np.mean(abs(bank[valid])<=30)),blocks_by_category=counts))
fig.suptitle('Observed operating envelope — selection uses flight logs only');fig.tight_layout();fig.savefig(O/'mission-envelope-inventory.png',dpi=160)
out=dict(scope='Exploratory mission-envelope inventory following pilot steering away from aerobatic priority. No candidate forecast errors used. Does not silently replace existing acceptance gates.',rules=dict(window_s=15,grid_s=15,all_samples_valid_and_airborne=True,min_groundspeed_m_s=2,exploratory_max_bank_deg=30,max_course_rate_p95_deg_s=15,moderate_turn_course_change_deg=30,low_power_max_throttle=.08,steady_climb_min_m_s=.3),limitations=['Bank is payload attitude, not measured canopy bank.','Ground course and speed include wind.','Categories describe motion and controls; they do not establish AUTO mode.','Thresholds are descriptive selection rules, not revised success criteria.'],summaries=summaries,blocks=rows)
(O/'mission-envelope-inventory.json').write_text(json.dumps(out,indent=2));print(json.dumps(summaries,indent=2))

