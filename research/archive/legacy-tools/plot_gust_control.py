"""Plot matched gust screening and, when available, native AUTO validation."""
import argparse, json, html
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();study=a.root/'gust-control'
fig,axs=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
for case,label in [('baseline','Current controller'),('slow_pitch_010','0.1 gain / 0.5 Hz pitch feedback')]:
 d=np.load(study/'screen'/f'{case}.npz')['trace']
 for ax,col in zip(axs,[3,2,1,5]):ax.plot(d[:,0],d[:,col]*(100 if col==5 else 1),label=label,lw=1)
for ax,label in zip(axs,['Payload pitch (deg)','Climb (m/s)','Height error (m)','Throttle (%)']):ax.set_ylabel(label);ax.grid(alpha=.25);ax.axvline(80,color='black',ls='--')
axs[0].legend();axs[-1].set_xlabel('Time (s); gusts stop at 80 s');fig.suptitle('Matched nonlinear screen: same initial state and gust history')
fig.savefig(study/'screen-comparison.png',dpi=150);plt.close(fig)
summary=json.loads((study/'screen/summary.json').read_text());rows=summary['rows'];cols=['case','pitch_std','climb_rms','height_rms','pitch_rate_rms','throttle_std']
def table(rows,cols):
 return '<table><tr>'+''.join('<th>'+html.escape(c)+'</th>' for c in cols)+'</tr>'+''.join('<tr>'+''.join('<td>'+ (f'{r[c]:.3f}' if isinstance(r[c],float) else html.escape(str(r[c])))+'</td>' for c in cols)+'</tr>' for r in rows)+'</table>'
body='<h1>Gust-driven longitudinal oscillation</h1><p>Research log — 11 October 2026. Same calibrated 9-DOF Trout Lake model, 1.758 kg, 22.2 V. Raw traces stay in Mission Planner.</p><h2>Is the controller creating the motion?</h2><p>Fixed throttle still gives 7.82° pitch standard deviation versus 9.45° with the original controller. Removing the 0.34 Hz forcing component lowers it to 4.16°. That ablation also reduces total gust energy; it is a resonance diagnostic, not an equal-power comparison.</p><h2>Target the slow mode</h2><p>Test body pitch-rate feedback with gain 0.1 and a 0.5 Hz low-pass. Throttle correction is −0.1 × filtered q, with q in rad/s and throttle as a fraction. Native correction is limited to ±0.25. Height feedforward and PI are unchanged. The filter matters: this is not the earlier broadband sign experiment.</p><img src="screen-comparison.png"><p>Truth-state diagnostic controller, not native SITL. Statistics use seconds 15–80; gusts stop at 80 s. Model, lateral loop and gust realization are matched.</p>'+table(rows,cols)
if (study/'two-pole-screen/summary.json').exists():
    second=json.loads((study/'two-pole-cutoff-check/summary.json').read_text())['rows']
    body+='<h2>Refinement: reject faster coupled modes</h2><p>The first-order candidate destabilizes the softer-roll linear model at lateral scaling 1.4. Two cascaded 1.5 Hz stages with gain 0.1 retain the slow-mode benefit while all seven plant cases remain stable at scaling 0.9, 1.15 and 1.4. This is a bounded local sensitivity check, not a global guarantee. Native validation is summarized below.</p>'+table(second,cols)
    fig,axs=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
    for case,label in [('baseline','Current controller'),('two_pole_hz_150','Two-stage candidate')]:
        d=np.load(study/('screen' if case=='baseline' else 'two-pole-cutoff-check')/f'{case}.npz')['trace']
        for ax,col in zip(axs,[3,2,1,5]):ax.plot(d[:,0],d[:,col]*(100 if col==5 else 1),label=label,lw=1)
    for ax,label in zip(axs,['Payload pitch (deg)','Climb (m/s)','Height error (m)','Throttle (%)']):ax.set_ylabel(label);ax.grid(alpha=.25);ax.axvline(80,color='black',ls='--')
    axs[0].legend();axs[-1].set_xlabel('Time (s); gusts stop at 80 s')
    fig.savefig(study/'two-pole-comparison.png',dpi=150);plt.close(fig)
    body+='<img src="two-pole-comparison.png"><img src="filter-and-poles.png"><p>The slow-mode poles move from −0.248 ± j2.254 to −0.658 ± j2.186: damping ratio rises from 0.109 to 0.288. Other slower height/integrator modes remain; these numbers describe the oscillatory mode, not total settling time.</p>'
    if (study/'selected-validation/summary.json').exists():
        body+='<h3>Different phase and 50% stronger gusts</h3>'+table(json.loads((study/'selected-validation/summary.json').read_text())['rows'],cols)
native=[]
for folder,label in [('viewer-auto-turbulence','Baseline gusts'),('gust-control/native-gust','Candidate gusts'),('gust-control/two-pole-native-gust','Two-stage gusts'),('viewer-auto-mission','Baseline calm'),('gust-control/native-calm','Candidate calm'),('gust-control/two-pole-native-calm','Two-stage calm')]:
 run=a.root/folder
 if not (run/'auto-clip.jsonl').exists():continue
 frames=[json.loads(s) for s in (run/'auto-clip.jsonl').read_text().splitlines()];t=np.array([f['time_s'] for f in frames]);t-=t[0];y=np.array([f['state'] for f in frames]);pitch=Rotation.from_quat(y[:,3:7]).as_euler('xyz',degrees=True)[:,1];th=np.array([f['throttle'] for f in frames])*100
 row=dict(case=label,pitch_std=float(pitch.std()),climb_rms=float(np.sqrt(np.mean(y[:,9]**2))),height_rms=float(np.sqrt(np.mean((-y[:,2]-90)**2))),throttle_std=float(th.std()),throttle_min=float(th.min()),throttle_max=float(th.max()))
 native.append(row)
 if label=='Baseline gusts':fig,axs=plt.subplots(4,1,figsize=(11,10),sharex=True,layout='constrained')
 if label in ('Baseline gusts','Two-stage gusts'):
  for ax,v in zip(axs,[pitch,-y[:,9],-y[:,2]-90,th]):ax.plot(t,v,label=label,lw=1)
if native:
 for ax,label in zip(axs,['Payload pitch (deg)','Climb (m/s)','Height error (m)','Throttle (%)']):ax.set_ylabel(label);ax.grid(alpha=.25)
 axs[0].legend();axs[-1].set_xlabel('Seconds from first-loiter approach (separate runs)');fig.suptitle('Native AUTO validation: identical gust generator, route and model')
 fig.savefig(study/'native-comparison.png',dpi=150);plt.close(fig)
 body+='<h2>Native AUTO validation</h2><p>55-second first-loiter entry excerpts, independently aligned by route event. Gust phases may differ because event timing changes. Full mission completion is checked separately using unchanged AUTO test gates.</p><img src="native-comparison.png">'+table(native,['case','pitch_std','climb_rms','height_rms','throttle_std','throttle_min','throttle_max'])
body+='<h2>Complete-mission and settled-loiter checks</h2>'
full=[]
for folder,label in [('viewer-auto-turbulence','Baseline gusts'),('gust-control/native-gust','First candidate gusts'),('gust-control/two-pole-native-gust','Selected gusts'),('viewer-auto-mission','Baseline calm'),('gust-control/two-pole-native-calm','Selected calm')]:
    path=a.root/folder/'gust-metrics.json'
    if not path.exists():continue
    m=json.loads(path.read_text())
    for seq,v in m['loiters'].items():full.append(dict(case=label,loiter=seq,**v))
    body+=f"<p>{html.escape(label)}: maximum waypoint height error {m['max_waypoint_height_error_m']:.2f} m; corner overshoot {m['max_corner_overshoot_m']:.2f} m. Backend errors: {html.escape(str(m['backend']['errors']))}.</p>"
if full:body+=table(full,['case','loiter','pitch_std_deg','pitch_rate_rms_deg_s','climb_rms_m_s','height_rms_m','throttle_std_pct'])
body+='<p>Settled loiter statistics exclude the first 20 seconds after reaching 60 m from the loiter center, then cover the rest of each loiter. Different route timing changes sampled gust phase. The exactly matched truth-state screen provides the controlled comparison; native runs verify the implemented controller and mission behavior. The synthetic uniform gust field is not measured atmospheric turbulence.</p>'
(study/'comparison.json').write_text(json.dumps(dict(screen=rows,native=native),indent=2)+'\n')
(study/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>Gust controller study</title><style>body{font:17px system-ui;max-width:1100px;margin:35px auto;padding:20px;color:#183042}img{width:100%}table{border-collapse:collapse;font-size:14px}td,th{padding:9px;border-bottom:1px solid #ccc;text-align:right}h2{margin-top:40px}</style>'+body)
