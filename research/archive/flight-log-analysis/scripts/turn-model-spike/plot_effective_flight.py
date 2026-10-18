from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
fig,axes=plt.subplots(4,4,figsize=(16,12),sharex=True)
for col,(day,start) in enumerate([('2026-01-03',450),('2026-01-03',508),('2026-02-15',326),('2026-02-15',3387)]):
 a=np.load(O/f'{day}-{start}-effective-flight.npz');t=a['t']-a['t'][0];p=a['model'];y=a['truth']
 measured=[np.rad2deg(y[:,0]),np.rad2deg(y[:,5]-y[0,5]),np.linalg.norm(y[:,2:5],axis=1),-y[:,4]]
 predicted=[np.rad2deg(p[:,0]),np.rad2deg(p[:,5]-p[0,5]),p[:,3],p[:,3]*np.sin(p[:,4])]
 for row in range(4):
  ax=axes[row,col];ax.plot(t,measured[row],color='#163b56',lw=2,label='Flight');ax.plot(t,predicted[row],color='#d46a26',lw=2,label='Candidate');ax.grid(alpha=.2)
  if col==0:ax.set_ylabel(['Payload bank (deg)','Air-course change (deg)','Estimated airspeed (m/s)','Climb rate (m/s)'][row])
  if row==3:ax.set_xlabel('Forecast time (s)')
 axes[0,col].set_title(('Hood River' if day.endswith('03') else 'Trout Lake')+f' | {start} s')
axes[0,0].legend();fig.suptitle('Coupled flight model: successes and misses',fontsize=18)
fig.tight_layout(rect=(0,0,1,.96));fig.savefig(O/'iteration-effective-flight.png',dpi=160);plt.close(fig)
p=O/'physics-development.html';s=p.read_text(encoding='utf-8');section='''<section id="coupled-flight"><h2>Coupled bank, speed and climb</h2><figure><img src="iteration-effective-flight.png" alt="Flight and coupled-model predictions of bank, air-relative course, airspeed and climb for four maneuvers"><figcaption>Three-second forecasts initialized from measured flight state, driven by recorded throttle and per-flight calibrated brake commands. One parameter set is fitted across 53 ordinary manual-flight windows. These are development checks, not a sealed test set. Airspeed and air-relative course use the stated wind estimates; course is not payload heading.</figcaption></figure><p>Coupling lift, drag and thrust to the bank response substantially improves the Hood River high-bank course prediction. At 450 seconds, bank RMS error is 8.3 degrees and final air-course error is 3.1 degrees. Trout Lake at 326 seconds still misses by 10.7 degrees bank RMS and 52.9 degrees final course. At 3387 seconds bank matches within 1.0 degree RMS, but climb error is 1.0 m/s RMS. Matching bank alone therefore does not establish a usable simulator.</p><p>The candidate remains unaccepted. Its response time reaches the lower fit bound and lift slope approaches the upper bound; fitted values are effective coefficients, not measured hardware properties. Next checks compare brake-threshold assumptions in the coupled model and test longer predictions before estimating achievable sustained turn radius. Source: <code>fit_effective_flight.py</code>; results: <code>effective-flight-fit.json</code>.</p></section>'''
if 'id="coupled-flight"' not in s:p.write_text(s.replace('</html>',section+'</html>'),encoding='utf-8')
