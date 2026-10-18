"""Export bounded swoop replays and an evidence-focused local research report."""
import json,html,subprocess
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
r=Path(__file__).parent
raw=Path('/mnt/c/Users/davin/OneDrive/Documentos/Mission Planner/analysis/swoop-2026-10-13')
viewer=r.parents[1]/'viewer/assets/recordings/landing'
selection=json.loads((r/'selection.json').read_text())
results=json.loads((r/'results.json').read_text())
for style,name in selection['replays'].items():
 d=raw/name;land=json.loads((d/'landing.json').read_text());frames=json.loads((d/'viewer.json').read_text());end=land['contact']['time']
 left=max((f for f in frames if f['time_s']<=end),key=lambda f:f['time_s']);right=min((f for f in frames if f['time_s']>=end),key=lambda f:f['time_s'])
 q=(end-left['time_s'])/(right['time_s']-left['time_s']) if right['time_s']>left['time_s'] else 0
 state=np.array(left['state'])+q*(np.array(right['state'])-left['state'])
 for offset in [3,16]:state[offset:offset+4]=Slerp([0,1],Rotation.from_quat([left['state'][offset:offset+4],right['state'][offset:offset+4]]))([q]).as_quat()[0]
 endpoint=dict(left,time_s=end,state=state.tolist())
 for key in ['rpm','throttle','voltage_v','brake_mm','wind_ned_m_s','joint_rotvec_rad','joint_rate_rad_s']:
  endpoint[key]=(np.array(left[key])+q*(np.array(right[key])-left[key])).tolist()
 clip=[f for f in frames if end-35<=f['time_s']<end]+[endpoint]
 data=dict(frames=clip,target_ned_m=land['target_ned_m'],result=land,label=f'{style.capitalize()} approach · native AUTO',
           flare_time_s=results[name]['flare_time'],parameters=results[name]['parameters'],
           selection='Frozen-style calm robustness run; first CG ground-plane crossing. High-brake physics is not flight validated.',
           flare_event_source='First active native PGFL sample')
 (viewer/f'swoop-{style}.json').write_text(json.dumps(data,separators=(',',':')))

names=selection['validation'];fig,axes=plt.subplots(1,3,figsize=(14,4.4))
for style,color in [('soft','#238b96'),('dramatic','#cf7436')]:
 selected=[(n,results[n]) for n in names if n.startswith(style)]
 for j,(n,x) in enumerate(selected):
  for ax,key in zip(axes,['miss','horizontal','sink']):ax.scatter(j+(0 if style=='soft' else .08),x[key],color=color,label=style if j==0 else None,s=50)
for ax,title,aspiration in zip(axes,['Target miss (m)','Horizontal touchdown speed (m/s)','Downward touchdown speed (m/s)'],[5,1,.5]):
 ax.axhline(aspiration,ls='--',color='gray',label='Aspiration');ax.set_title(title);ax.set_xticks(range(4),['Calm','Gust','Crosswind','Entry error']);ax.set_ylim(bottom=0);ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.suptitle('Frozen settings: native SITL robustness cases, not a calibrated probability distribution');fig.tight_layout();fig.savefig(r/'validation.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(2,2,figsize=(12,8))
for style,name in selection['replays'].items():
 d=raw/name;land=json.loads((d/'landing.json').read_text());frames=json.loads((d/'viewer.json').read_text());frames=[f for f in frames if f['time_s']<=land['contact']['time']]
 t=np.array([f['time_s'] for f in frames]);s=np.array([f['state'] for f in frames]);target=np.array(land['target_ned_m']);time=t-results[name]['flare_time']
 line,=axes[0,0].plot(s[:,1]-target[1],s[:,0]-target[0],label=style);color=line.get_color()
 axes[0,1].plot(time,-s[:,2],label=style,color=color)
 axes[1,0].plot(time,np.linalg.norm(s[:,7:9],axis=1),label=f'{style}: horizontal',color=color)
 axes[1,0].plot(time,s[:,9],ls='--',label=f'{style}: downward',color=color)
 rotation=Rotation.from_quat(s[:,3:7]);rates=np.rad2deg(rotation.inv().apply(s[:,10:13]))
 axes[1,1].plot(time,rates[:,1],label=style,color=color)
axes[0,0].scatter(0,0,marker='x',color='red');axes[0,0].set_aspect('equal',adjustable='datalim');axes[0,0].set(xlabel='East of target (m)',ylabel='North of target (m)',title='Calm replay tracks')
axes[0,1].set(xlabel='Time relative to flare (s)',ylabel='Height (m)',title='Approach and rollout')
axes[1,0].set(xlabel='Time relative to flare (s)',ylabel='Speed (m/s)',xlim=(-3,2.5),title='Speed reduction and transient sink response')
axes[1,1].set(xlabel='Time relative to flare (s)',ylabel='Payload pitch rate (deg/s)',xlim=(-3,2.5),title='Remaining motion at crossing')
for ax in axes.flat:ax.grid(alpha=.2);ax.legend()
fig.tight_layout();fig.savefig(r/'styles.png',dpi=150);plt.close(fig)
rows=[]
for n in names:
 x=results[n];rows.append(f"| {n} | {x['miss']:.2f} | {x['horizontal']:.2f} | {x['sink']:.2f} | {x['bank']:.1f} | {x['body_rate_dps'][1]:.1f} |")
md='''# Native turning-descent and flare demonstration

The simulator now completes a tight right-turning descent, rollout and brake flare with two approach settings. This is a precision turning-landing demonstration, **not yet a settled, near-zero-speed swoop**. The report retains the remaining forward speed and pitch motion rather than equating target accuracy with touchdown quality.

## Watch the native runs

[Soft approach](../../viewer/dist/landing.html?case=swoop-soft) · [Dramatic approach](../../viewer/dist/landing.html?case=swoop-dramatic)

The recordings use native ArduPilot outputs, the unchanged 9-DOF Trout Lake model and actual simulated gusts. Nothing is animated into the motion. The floor and bullseye are earth-fixed; the flight path is shown in 3D, and the altitude profile uses distance traveled through the turn. The animation ends at first payload-CG ground crossing, not a modeled physical landing or slide.

## Architecture and style settings

Both versions use AUTO waypoint guidance, the existing heading-rate controller, throttle-only parafoil TECS and native AP_Landing scheduled common-brake flare. No MPC or firmware changes were required for this demonstration. A short research harness specifies the mission and, for the dramatic style, changes the L1 period using native estimated heading.

| Setting | Soft | Dramatic |
|---|---:|---:|
| Entry / corner altitude | 25 / 10 m | 20 / 8 m |
| Distance to corner / final leg | 55 / 40 m | 45 / 40 m |
| Heading-rate command limit | 25 deg/s | 45 deg/s |
| L1 period | 10 s | 6 s, restored to 10 s after estimated heading passes 65° |
| L1 damping / measurement cutoff | 1.0 / 1 Hz | 1.0 / 1 Hz |
| Flare trigger | 1.1 m | 1.1 m |
| Common-brake target / reserve | 85% / 10% | 95% / 5% |
| Flare ramp | 100 percentage points/s | 100 percentage points/s |

All other selected inner lateral and longitudinal gains are unchanged. These are tested presets, not a validated continuum obtained by interpolating controller gains. The dramatic schedule handles heading wrap at north; the earlier incorrect schedule and its affected recordings remain classified as development evidence, not final validation.

![Tracks, height, flare velocities and residual pitch motion](styles.png)

## Frozen-style robustness results

The four cases per style are calm air; synthetic gusts of 0.25 m/s horizontal amplitude and one-quarter that vertically; a 1 m/s northerly velocity component, crosswind to the eastbound final leg; and a combined entry perturbation (2 m lateral offset, +1 m height, +5° heading, +5% initial speed, 150 mm/s actuator rate and 60 ms command delay). These are engineering checks, not statistical reliability estimates.

| Run | Miss (m) | Horizontal speed (m/s) | Downward speed (m/s) | Bank at crossing (deg) | Pitch rate (deg/s) |
|---|---:|---:|---:|---:|---:|
'''+ '\n'.join(rows)+'''

![Precision and speed objectives across the eight final cases](validation.png)

The aspirational 1 m/s horizontal and 0.5 m/s downward speeds were not achieved together. The flare produces a transient reduction in sink, but the forward-speed minimum occurs later and is accompanied by renewed sink and payload pitch motion. Simply increasing brake or changing the trigger cannot independently set position, both velocities and swing phase.

## Why the study stops here

The open-loop authority screen examined 20 turning/flare combinations, followed by 12 faster-flare sensitivity cases. It is a feasibility screen, not proof that no better control law exists. It found useful transient braking but did not demonstrate a near-stop touchdown. Full brake in this screen is an extrapolation: the model uses empirical lift and drag versus brake with a smooth angle-of-attack lift rolloff; it has no validated high-brake stall or canopy-collapse behavior.

![Transient authority with fixed actuator limits](fast-flare.png)

This bounded result supports continued terminal-control work, but does not justify replacing all of TECS with MPC. The next useful comparison is a short optimized brake schedule with explicit pitch/swing penalties against a small terminal predictive controller. Both must use the same model, entry states and actuator limits. Real high-brake measurements remain necessary to establish whether the predicted recovery arc and flare are physical.

## Reproduction

`run.py` and `native.py` run the named native cases; `analyze.py` audits logs and computes crossing metrics; `publish.py` exports bounded replays and plots. `selection.json` identifies the final cases. `provenance.json` pins the unchanged simulator and native binary. Raw logs, full recordings, intermediate numerical traces and browser screenshots remain in Mission Planner/analysis/swoop-2026-10-13. Run the viewer build, then serve the tinkering repository over localhost to open the report and replay links.

Implementation and analysis assisted by Codex.
'''
final=[results[n] for n in names]
summary=f"All eight final runs finish within 5 m; worst miss is {max(x['miss'] for x in final):.2f} m. Horizontal crossing speeds span {min(x['horizontal'] for x in final):.2f}–{max(x['horizontal'] for x in final):.2f} m/s; downward speeds span {min(x['sink'] for x in final):.2f}–{max(x['sink'] for x in final):.2f} m/s. Payload pitch-rate magnitude reaches {max(abs(x['body_rate_dps'][1]) for x in final):.1f} deg/s at crossing. These velocities and residual motion fall short of the settled touchdown objective."
md=md.replace('## Frozen-style robustness results',summary+'\n\n## Frozen-style robustness results')
md=md.replace('The aspirational 1 m/s', 'The dramatic gust and crosswind cases spend about a quarter of the flare at the remaining differential-command limit. The bank remains small at crossing, but the narrow steering reserve is a real tradeoff. The [complete metrics](results.json) include physical travel limits, body rates and canopy–payload relative rotation and rate.\n\nThe aspirational 1 m/s')
md=md.replace('All other selected inner', 'In the calm recordings, the soft turn reaches roughly 11° of bank and the dramatic turn roughly 19°. The dramatic turn accelerates from about 3.7 to 4.5 m/s airspeed between 30° and 90° heading, close to ordinary flight speed rather than a sustained high-energy dive.\n\nAll other selected inner')
(r/'report.md').write_text(md)
# Small standalone report; the repository server serves the already-built applets.
body=subprocess.check_output(['/home/david/.npm/_npx/52027bd8fc0022aa/node_modules/node/bin/node','-e',"const {marked}=require('/home/david/repos/DavidIngraham.github.io/assets/vendor/marked.js');process.stdout.write(marked.parse(require('fs').readFileSync(0,'utf8')));"],input=md,text=True)
body=body.replace('<h2>Watch the native runs</h2>','<h2>Watch the native runs</h2><iframe title="Soft approach" src="../../viewer/dist/landing.html?case=swoop-soft"></iframe><iframe title="Dramatic approach" src="../../viewer/dist/landing.html?case=swoop-dramatic"></iframe>')
(r/'report.html').write_text('<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Swoop landing study</title><style>body{max-width:1100px;margin:30px auto;padding:0 20px;font:18px/1.6 system-ui;color:#203440;background:#faf9f5}img{max-width:100%}iframe{width:100%;height:1050px;border:0;background:#102330}table{border-collapse:collapse;font-size:15px;display:block;overflow:auto}td,th{padding:7px 12px;border-bottom:1px solid #ccd2d8}a{color:#237783}code{font-size:15px}</style>'+body)
print('Exported final replay selections:',selection['replays'])