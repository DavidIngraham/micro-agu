"""Summarize the rotor-inclusive longitudinal controls experiment."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.linalg import solve_continuous_lyapunov
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(sys.argv[1]);src=Path(__file__).with_name('longitudinal_classical.py').read_text();defs=src[src.index('def battery_scaling'):src.index('cases=')];ns=dict(np=np);from scipy.signal import tf2ss
ns.update(tf2ss=tf2ss,kv=.6/35,ki=.3*.6/35,kh=.1);exec(defs,ns);mechanical,close=ns['mechanical'],ns['close']
rows=json.loads((root/'summary.json').read_text());checks=json.loads((root/'nonlinear-summary.json').read_text());slopes=json.loads((root/'trim-slopes.json').read_text());modes=json.loads((root/'modes.json').read_text());val=json.loads((root/'sine-validation.json').read_text());scan=[]
for name in ['level','climb','descent','low_voltage','dynamic_roll','dynamic_climb']:
    p=np.load(root/name/'plant.npz');A,B,C=p['A'],p['B'],p['C'];speed=next(r for r in rows if r['case']==name)
    # Reconstruct frozen speed scaling from trim state rather than assumed airspeed.
    trim=np.load(root/name/'trim.npz');S=5/np.linalg.norm(trim['state'][7:10]);am,bm,cm=mechanical(A,B,C,S,'4');gff=.18 if name=='descent' else .15
    for kind,sign in [('body',1),('washed',1),('relative',-1)]:
        for gain in [0,.005,.01,.02,.05,.1,.2]:
            ac,bc,cc=close(am,bm,cm,gff,kind,sign*gain,speed['voltage']);stable=max(np.linalg.eigvals(ac).real)<0
            energy=None
            if stable:
                Q=solve_continuous_lyapunov(ac.T,-np.outer(cc[1],cc[1]));x=np.zeros(len(ac));x[9]=np.deg2rad(1);energy=float(x@Q@x)*(180/np.pi)**2
            scan.append(dict(case=name,kind=kind,gain=sign*gain,stable=bool(stable),pitch_energy_deg2_s=energy))
(root/'gain-scan.json').write_text(json.dumps(scan,indent=2))
fig,axes=plt.subplots(1,3,figsize=(15,5))
for ax,kind in zip(axes,['body','washed','relative']):
    for name in ['level','climb','descent','low_voltage','dynamic_roll','dynamic_climb']:
        rr=[r for r in scan if r['case']==name and r['kind']==kind];base=rr[0]['pitch_energy_deg2_s'];ax.plot([abs(r['gain']) for r in rr],[r['pitch_energy_deg2_s']/base if r['stable'] else np.nan for r in rr],'o-',label=name)
    ax.axhline(1,color='gray',ls='--');ax.set(title=kind+(' (negative gain)' if kind=='relative' else ' (positive gain)'),xlabel='|Damper gain| throttle / (rad/s)',ylabel='Pitch-kick energy / damper-off');ax.grid(alpha=.25)
axes[0].legend(fontsize=7);fig.suptitle('Bounded gain scan: integrated pitch-rate energy after a 1 deg/s kick; 4 Hz lateral feedback');fig.tight_layout();fig.savefig(root/'gain-scan.png',dpi=150);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(13,5))
for label in ['off','wrong sign','reversed body','washed body','relative −']:
    rr=[r for r in checks if r['damper']==label];xx=[r['case']+' / '+r['lateral']+' Hz' for r in rr]
    axes[0].plot(xx,[r['pitch_kick_rms_deg_s'] for r in rr],'o-',label=label);axes[1].plot(xx,[r['climb_height_rmse'] for r in rr],'o-',label=label)
for ax in axes:ax.tick_params(axis='x',rotation=25);ax.grid(alpha=.25)
axes[0].set(ylabel='Pitch rate RMS during first 40 s (deg/s)',title='Disturbance recovery');axes[1].set(ylabel='Height error RMS after climb command (m)',title='Command tracking');axes[0].legend(fontsize=8);fig.tight_layout();fig.savefig(root/'nonlinear-summary.png',dpi=150);plt.close(fig)
report='''<!doctype html><meta charset="utf-8"><title>Longitudinal linear controls</title><style>body{font:17px/1.55 system-ui;max-width:1150px;margin:32px auto;padding:0 20px;color:#18333e}img{width:100%}table{border-collapse:collapse;font-size:14px}td,th{padding:8px;border-bottom:1px solid #ccc;text-align:left}pre{white-space:pre-wrap;background:#eef3f6;padding:18px}</style><h1>Longitudinal modes, throttle feedback, and what to improve next</h1><p>11 October 2026 · 9-DOF articulated parafoil + causal rotor RPM · native controller algebra in a standalone diagnostic harness</p>
<p>Two questions are separate: how to damp pitch motion, and how to command the right throttle for a climb. The damper acts on fast motion; the slow altitude loop and voltage-dependent thrust calibration need their own treatment. This study compares signs and filters without changing the native flight-controller implementation.</p>
<h2>Model and operating points</h2><p>The plant retains 14 internal mechanical states plus rotor RPM. Global position and heading are removed. Height error is then added explicitly to close the altitude loop. Four mission trims are studied: level, +0.5 m/s climb, −0.5 m/s descent, and level at 20.4 V. The dynamic-roll profile is checked at level and climb. Level is evaluated with both one-sided feedforward slopes because the native law changes slope at zero demanded climb. For each case, brakes are fixed or driven by the current 4 Hz lateral controller or proposed 1 Hz alternative.</p>
<p>Outputs are CG climb rate, payload body pitch rate, and relative canopy angular velocity projected on the payload pitch axis. The relative-rate signal is simulation truth, not an implemented observer. The nonlinear motor/propeller path includes changing RPM, thrust, reaction torque and rotor angular momentum. The added level-flight rotor pole is about −56.94 s⁻¹ (17.6 ms time constant). The mechanical poles retain coupled motion; the slow pitch/speed pair is approximately −0.338 ± j2.154 s⁻¹ (0.343 Hz).</p>
<h2>Native longitudinal law, locally unsaturated</h2><pre>e_h = h_reference − h;       ė_h = v_reference − v
r_f(s) = v_reference(s)/(0.5s + 1)
v_des = e_h/10 + r_f
u_TECS = u_ff(v_des) + K_v(v_des − v) + I + k H_d(s) q_signal
u_motor = c(V) u_TECS; c(V) = 22.2 / clamp(V, 18, 22.2)
İ = K_i(v_des − v)
K_v = 0.6/(5×7) = 0.0171429; K_i = 0.3 K_v = 0.00514286
u_ff = 0.45 + 0.15 v_des (climb), or 0.45 + 0.18 v_des (descent)

Body: H_d = 62.832/(s + 62.832)
Washed / relative: H_d = [18.850/(s + 18.850)] [s/(s + 3.770)]

q_signal = q_payload, or [(ω_canopy − ω_payload) in payload axes]_y
k > 0 adds throttle for positive measured rate.
Native TECS_PTCH_DAMP = −k for payload-rate feedback.</pre><p>These numeric gains apply at 22.2 V. At 20.4 V native servos.cpp also feeds its compensated integer-percent maximum (65%) back into TECS: K_v becomes 0.65/35, K_i=0.3 K_v, and the climbing feedforward slope becomes 0.65−0.45=0.20 before downstream scaling by 22.2/20.4. This interaction is included in the linear and nonlinear comparisons.</p>
<p>The outer height correction also enters feedforward, so the local feedforward slope contributes to feedback stability. Native command limits, throttle clipping/slew, integrator bounds, rate filtering and 0.5 s feedforward smoothing are represented in the nonlinear harness. It uses 200 Hz truth observations, not the native scheduler or EKF. Integrators start at the exact trim bias; low-voltage runs therefore test local dynamics after trim compensation, not cold-start recovery from an incorrect cruise-throttle setting.</p>
<h2>Plant and closed-loop frequency response</h2><img src="plant-bode.png"><img src="closed-bode.png"><p>The brake-fixed plant isolates throttle actuation. Active lateral feedback changes the response around coupled modes. The closed-loop curves include the height-error integrator, vertical-speed PI and feedforward smoothing; raw pitch feedback and relative feedback are not interchangeable.</p>
<h2>Damper root loci</h2><img src="damper-root-locus.png"><p>Gain is swept from −0.3 to +0.3 throttle fraction per rad/s. Black crosses show damping off; green and red mark +0.02 and −0.02. Only the plotted region is shown. Filter poles and the slow altitude poles are included in the eigenvalue calculation. Positive gain corresponds to the reversed payload-rate sign; relative-rate feedback is evaluated in both directions independently.</p>
<h2>Bounded gain exploration</h2><img src="gain-scan.png"><p>The energy integral is calculated from a continuous Lyapunov equation for a 1 deg/s payload pitch-rate perturbation with joint-compatible velocity. It includes the full stable closed loop, not only a selected pitch pair. Missing points denote unstable cases. This is a disturbance-recovery metric, not a command-tracking or flight-readiness score; the nonlinear checks below use the previously tested magnitude 0.02 rather than selecting a large gain from this one metric.</p>
<h2>Nonlinear checks</h2><img src="nonlinear-dampers.png"><img src="nonlinear-summary.png"><p>Twenty-five 90-second runs compare five alternatives. A 1 deg/s payload pitch-rate kick starts each run; a +0.1 m/s climb increment begins at 40 s. Both lateral filters are tested at level; climb, low voltage and the alternate roll profile use 1 Hz lateral filtering. No wind, launch, landing or estimator effects are included. Damper clipping is ±0.25 throttle and the physical throttle limit is 60% at 22.2 V and 65% at 20.4 V, matching native integer-percent limit scaling.</p><table><tr><th>Case / lateral</th><th>Damper</th><th>Kick pitch RMS deg/s</th><th>Climb height RMS m</th><th>Throttle saturation fraction</th></tr>'''
for r in checks:report+=f"<tr><td>{r['case']} / {r['lateral']} Hz</td><td>{r['damper']}</td><td>{r['pitch_kick_rms_deg_s']:.5f}</td><td>{r['climb_height_rmse']:.5f}</td><td>{r['saturation_fraction']:.3f}</td></tr>"
report+='</table><h2>Trim and feedforward</h2><table><tr><th>Case</th><th>Voltage</th><th>Trim throttle</th><th>Inverse physical-throttle climb gain</th><th>Inverse pre-compensation command gain</th></tr>'
for r in slopes:report+=f"<tr><td>{r['case']}</td><td>{r['voltage']:.1f}</td><td>{r['trim_throttle']*100:.2f}%</td><td>{r['inverse_climb_gain']:.4f}</td><td>{r['inverse_command_gain']:.4f}</td></tr>"
report+='''</table><p>Dropping from 22.2 to 20.4 V raises level-flight trim throttle from 44.59% to 48.93%. The local inverse climb gain rises from 0.1644 to 0.1804. ArduPlane already provides downstream voltage compensation. It was disabled in the earlier native tests and omitted from the first standalone analysis. It is now enabled using a 22.2 V calibration reference, with the simulator battery voltage connected to the flight controller. The required inverse command slope at 20.4 V is approximately 0.1658 after output scaling. Native TECS also changes its climb slope when the output limit changes; its implemented behavior, rather than an ideal feedforward correction, is modeled here.</p>
<h2>Numerical validation and provenance</h2>'''
maximum=max(max(r['relative_complex_error']) for r in val)
report+=f'<p>Eight nonlinear sine checks (0.1, 0.33, 1 and 3 Hz; amplitudes 0.0005 and 0.001) agree with the sampled linear model to within {100*maximum:.4f}% relative complex error across climb, payload pitch and relative pitch. Finite-difference step-size convergence is retained in summary.json. This verifies simulator linearization, not real-aircraft identification.</p>'
report+='''<p>Source: ArduPilot mechanics/controller revision af6b2ca963 plus simulator battery-compensation configuration and log-capture update c441b937bd. Tools/longitudinal_classical.py and research/archive/legacy-tools/report_longitudinal_classical.py in the tinkering repository reproduce this study; the chart initialization is reused from research/archive/legacy-tools/classical_lateral.py. Raw matrices and traces remain in Mission Planner. The mechanics are unchanged; all new controllers are diagnostic research comparisons.</p><pre>python3 research/archive/legacy-tools/longitudinal_classical.py --model PATH/Tools/simulation/paraglider --output PATH/classical-longitudinal
python3 research/archive/legacy-tools/report_longitudinal_classical.py PATH/classical-longitudinal</pre><p><a href="summary.json">Linear metrics</a> · <a href="nonlinear-summary.json">Nonlinear metrics</a> · <a href="modes.json">Modal participation</a> · <a href="gain-scan.json">Gain scan</a></p>'''
(root/'report.html').write_text(report)
print('Report written; worst sine error percent',maximum*100)

# Resolve the slow pitch pair and damper stability boundaries directly.
from scipy.optimize import brentq
boundaries=[];mode_rows=[]
fig,ax=plt.subplots(1,2,figsize=(12,5))
for axis,name in zip(ax,['level','climb']):
    pp=np.load(root/name/'plant.npz');tt=np.load(root/name/'trim.npz');S=5/np.linalg.norm(tt['state'][7:10]);am,bm,cm=mechanical(pp['A'],pp['B'],pp['C'],S,'4')
    for label,kind,gain in [('off','body',0),('reversed body','body',.02),('washed body','washed',.02),('relative −','relative',-.02)]:
        eig=np.linalg.eigvals(close(am,bm,cm,.15,kind,gain)[0]);q=eig[np.argmin(abs(eig-(-.3+2.15j)))];mode_rows.append(dict(case=name,damper=label,pole=[q.real,q.imag],damping_ratio=float(-q.real/abs(q))));axis.plot(q.real,q.imag,'o',label=label);axis.annotate(label,(q.real,q.imag),xytext=(4,5),textcoords='offset points',fontsize=8)
    axis.axvline(0,color='black');axis.set(title=name+' / 4 Hz lateral',xlabel='Real part (1/s)',ylabel='Imaginary part (rad/s)',xlim=(-.6,.02),ylim=(1.9,2.35));axis.grid(alpha=.25)
fig.suptitle('Slow pitch/speed mode: reversed body-rate damping moves it toward instability');fig.tight_layout();fig.savefig(root/'slow-pitch-poles.png',dpi=150);plt.close(fig)
for name in ['level','climb','descent','low_voltage','dynamic_roll','dynamic_climb']:
    pp=np.load(root/name/'plant.npz');tt=np.load(root/name/'trim.npz');S=5/np.linalg.norm(tt['state'][7:10])
    for lateral in ['4','1']:
        am,bm,cm=mechanical(pp['A'],pp['B'],pp['C'],S,lateral);gff=.18 if name=='descent' else .15
        for kind,sign in [('body',1),('washed',1),('relative',-1)]:
            def mx(k):return max(np.linalg.eigvals(close(am,bm,cm,gff,kind,sign*k,next(r['voltage'] for r in rows if r['case']==name))[0]).real)
            gains=np.linspace(0,.3,151);value=None
            for lo,hi in zip(gains[:-1],gains[1:]):
                if mx(lo)<0 and mx(hi)>=0:value=sign*brentq(mx,lo,hi);break
            boundaries.append(dict(case=name,lateral=lateral,kind=kind,first_unstable_gain=value))
(root/'boundaries.json').write_text(json.dumps(boundaries,indent=2));(root/'slow-pitch-modes.json').write_text(json.dumps(mode_rows,indent=2))
conclusion='''<h2>Decision</h2><p><strong>Keep pitch damping off in the native candidate; retain the existing native voltage compensation and verify its effect in SITL.</strong> Reversed body-rate feedback reduces the fast pitch-kick response, but moves the slow pitch/speed mode toward instability. At level with the current lateral controller, the 40-second kick RMS falls from 0.03358 to 0.03098 deg/s (7.7%), while the later climb-command ringing lasts longer. With the 1 Hz lateral filter in climbing flight, final-window pitch-rate standard deviation rises from 0.00624 to 0.09367 deg/s. Both are small absolute values, but the direction is consistent with the pole movement.</p><p>Washout reduces the slow-mode penalty. Relative-rate feedback, using the independently checked negative sign, also avoids most of it, but its disturbance benefit is modest and it assumes a measurement we do not currently have. Neither result justifies adding an observer or selecting a damper for the native mission controller yet. The 4.34 percentage-point increase in level trim throttle at 20.4 V is largely supplied by the existing compensation. This corrected analysis includes that path and the native TECS limit feedback; no new feedforward controller is proposed.</p><img src="slow-pitch-poles.png"><p>The scan is intentionally bounded. It shows why a single “correct sign” cannot damp every coupled mode, rather than presenting the largest numerically stable gain as a tuning recommendation.</p>'''
target=root/'report.html';report=target.read_text();report=report.replace('<h2>Model and operating points</h2>',conclusion+'<h2>Model and operating points</h2>');target.write_text(report)
print('SLOW MODES',mode_rows)
print('BODY GAIN BOUNDARIES',[r for r in boundaries if r['kind']=='body'])

report=(root/'report.html').read_text()
report=report.replace('<h2>Damper root loci</h2>','<h2>Damper root loci</h2><p>With current 4 Hz lateral feedback, positive payload-rate gain first destabilizes the local level-flight model near k=0.0688, but climbing flight reaches that boundary near k=0.04235. The tested k=0.02 is below both boundaries, yet already reduces slow-mode damping: in climb, the pitch-pair damping ratio falls from 0.0742 to 0.0387. This explains longer ringing without implying that the tested controller is unstable.</p>')
report=report.replace('<h2>Plant and closed-loop frequency response</h2>','<h2>Plant and closed-loop frequency response</h2><pre>G_v(s) = C_v(sI − A)⁻¹ B_throttle\nG_q(s) = C_q(sI − A)⁻¹ B_throttle\nG_rel(s) = C_rel(sI − A)⁻¹ B_throttle</pre><p>The saved plant.npz in each case contains A, the throttle and brake columns of B, the output matrix C, and the sampled realization. With active steering, its filter and integrator states are included before closing the altitude loop. The full order is retained.</p>')
(root/'report.html').write_text(report)
if '--archive' in sys.argv:
    import hashlib,shutil
    repo=Path(__file__).resolve().parents[1];archive=repo/'research/archive/flight-log-analysis';mp=archive/'manifest.json';manifest=json.loads(mp.read_text());entries={r['path']:r for r in manifest['files']};gallery=archive/'index.html';page=gallery.read_text();cards=[]
    for file in root.glob('*.png'):
        rel='figures/turn-model-spike/classical-longitudinal/'+file.name;dest=archive/rel;dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(file.read_bytes());dest.chmod(0o644)
        entries[rel]=dict(path=rel,source=str(file),bytes=dest.stat().st_size,sha256=hashlib.sha256(dest.read_bytes()).hexdigest(),kind='figure',referenced_by=['turn-model-spike/'+root.name+'/report.html'])
        if rel not in page:cards.append(f'<figure><a href="{rel}"><img loading="lazy" src="{rel}" alt="{file.stem}"></a><figcaption>{rel}</figcaption></figure>')
    manifest['files']=sorted(entries.values(),key=lambda r:r['path']);mp.write_text(json.dumps(manifest,indent=2)+'\n');gallery.write_text(page.replace('</main>',''.join(cards)+'</main>'))