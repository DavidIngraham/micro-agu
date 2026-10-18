from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;rows=[];mode_summary=[];fig,axs=plt.subplots(2,1,figsize=(12,7))
for ax,(day,name) in zip(axs,[('2026-01-03','2026-01-03 15-41-42'),('2026-02-15','2026-02-15 15-38-50')]):
 a=np.load(O/f'{day}-causal-input.npz');b=np.load(O/f'{day}.npz');p=np.load(O/f'{day}-propulsion-9x45.npz');t=a['t'];y=a['Y'];air=np.interp(t,b['t'],b['airborne'].astype(float))>.99;valid=air&a['valid'].astype(bool);th=a['U'][:,0];m=json.loads((O.parent/name/'MODE.json').read_text());mt=np.array([r['TimeUS']/1e6 for r in m]);mi=np.searchsorted(mt,t,side='right')-1;mode=np.array([r.get('ModeNum',r['Mode']) for r in m])[np.maximum(mi,0)];mode[mi<0]=-1
 for num in np.unique(mode[valid]):mode_summary.append(dict(day=day,mode_num=int(num),valid_airborne_s=float(np.sum(valid&(mode==num))*.04)))
 mask=valid&(th<=.08);edges=np.diff(np.r_[False,mask,False].astype(int));starts=np.where(edges==1)[0];ends=np.where(edges==-1)[0]
 ax.plot(t,np.where(valid,-y[:,8],np.nan),lw=.7,label='Climb rate');ax.plot(t,np.where(valid,th*5,np.nan),lw=.7,label='Throttle × 5');ax.set(title=day,xlabel='Log time (s)',ylabel='Climb (m/s) / scaled throttle');ax.grid(alpha=.2)
 for i,j in zip(starts,ends):
  duration=(j-i)*.04
  if duration<3:continue
  row=dict(day=day,start_s=float(t[i]),duration_s=duration,mode_nums=np.unique(mode[i:j]).tolist(),mean_sink_m_s=float(np.mean(y[i:j,8])),median_rpm=float(np.median(p['rpm'][i:j])),max_throttle=float(np.max(th[i:j])),max_abs_bank_deg=float(np.rad2deg(np.max(abs(y[i:j,0])))),mean_groundspeed_m_s=float(np.mean(np.linalg.norm(y[i:j,6:8],axis=1))))
  rows.append(row);ax.axvspan(t[i],t[j-1],color='green',alpha=.2)
 ax.legend(fontsize=8)
fig.suptitle('Low-power segments (green): at least 3 s, throttle ≤ 8%, valid airborne samples');fig.tight_layout();fig.savefig(O/'low-power-segments.png',dpi=160)
(O/'low-power-segments.json').write_text(json.dumps(dict(scope='Observed low-power opportunities, not identified equilibrium glide. RPM, bank, mode changes and proximity to landing must be reviewed.',segments=rows,mode_occupancy=mode_summary),indent=2));print(json.dumps(rows,indent=2))
