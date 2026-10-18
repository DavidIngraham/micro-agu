from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent
r=json.loads((O/'rpm-response-fit.json').read_text())
a=np.load(O/'rpm-response-development.npz')
fig,axes=plt.subplots(3,2,figsize=(12,9),sharex=True)
for ax,i,title in zip(axes[:,0],[0,4,5],['Hood River: powered turn, 447 s','Trout Lake: sustained input, 402 s','Trout Lake: throttle increase, 3387 s']):
 ax.plot(a['t'],a['truth'][i],color='#202d3a',label='ESC telemetry (scoring interpolation)')
 ax.plot(a['t'],a['model'][i],color='#e58228',label='Predicted RPM')
 ax.axvspan(0,.32,color='gray',alpha=.12)
 ax.set(title=title,ylabel='Motor RPM');ax.grid(alpha=.2)
for ax,i in zip(axes[:,1],[0,4,5]):
 ax.plot(a['t'],a['controls'][i]*100,color='#347ca5',label='Logged throttle')
 ax.set(ylabel='Throttle (%)');ax.grid(alpha=.2)
 other=ax.twinx();other.plot(a['t'],a['voltage'][i],color='#9b5f9c',label='Measured voltage');other.set_ylabel('Battery voltage (V)',color='#9b5f9c')
axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8)
for ax in axes[-1]:ax.set_xlabel('Seconds after initialization')
fig.suptitle('Motor speed is much better constrained than canopy dynamics',fontsize=16)
fig.text(.5,.025,'Measured voltage supplied throughout. Shaded first 0.32 s excluded from scoring. This does not validate thrust or the full flight model.',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.05,1,.95));fig.savefig(O/'iteration-rpm-response.png',dpi=150);plt.close(fig)
rows=[v for v in r['validation'] if v['group']=='remaining']
summary={}
for d in sorted(set(v['day'] for v in rows)):
 rr=[v for v in rows if v['day']==d]
 summary[d]={'windows':len(rr),'median_rmse_rpm':float(np.median([v['rpm_rmse'] for v in rr])),'p90_rmse_rpm':float(np.percentile([v['rpm_rmse'] for v in rr],90)),'median_relative_rmse_percent':float(np.median([100*v['rpm_rmse']/v['rpm_mean'] for v in rr if v['rpm_mean']>1000]))}
(O/'rpm-response-summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
p=O/'physics-development.html';h=p.read_text(encoding='utf-8');marker='id="rpm-response-discovery"'
section='''<section id="rpm-response-discovery"><h2>ESC telemetry constrains motor response</h2><p>Both main flights logged motor RPM, voltage, and current. A shared throttle-and-voltage response fitted on 62 windows predicts RPM on the remaining 254 windows with a median error of about 89 RPM. These are internal comparison windows, not a pristine final validation set.</p><img src="iteration-rpm-response.png" alt="Measured and predicted motor RPM in three turn windows with throttle and battery voltage" style="max-width:100%"><p>The fitted steady response is RPM = 743.58 V max(throttle − 0.00051, 0)<sup>0.90868</sup>. This is an effective loaded response, not the motor's nameplate KV. The fitted 0.018 s time constant is shorter than the telemetry sampling interval and is not a resolved mechanical time constant. Voltage is supplied from the log throughout these replays.</p><p>An official APC 9×6E performance map provides a provisional route from RPM and inflow to thrust, but its predicted shaft power exceeds measured electrical input in Trout Lake. That energy inconsistency must be resolved before using the map as calibrated thrust. The CAD propeller designation alone does not establish the exact flown propeller. Hood River's known faulty avionics current readings are excluded from any power calibration.</p><p>The pilot also confirmed a rigging adjustment. Its timing, side, and magnitude remain unspecified. It should be represented as a possible configuration change, rather than silently absorbed into common aerodynamic coefficients. Per-flight logged PWM limits remain authoritative for commanded travel.</p><p>The full flight model still fails the demanding turn checks. Better motor-speed prediction is a useful component result, not a successful full-aircraft fit.</p><p>Reference: <a href="https://www.apcprop.com/technical-information/performance-data/">APC computed performance data</a>; <a href="https://www.apcprop.com/files/PER3_9x6E.dat">9×6E data table</a>.</p></section>'''
if marker not in h:
 h=h.replace('</body>',section+'\n</body>') if '</body>' in h else h.replace('</html>',section+'\n</html>')
 p.write_text(h,encoding='utf-8')

