"""Write the local classical lateral-controls research report from computed results."""
import argparse
import html
import json
from pathlib import Path
import numpy as np
p=argparse.ArgumentParser();p.add_argument('directory',type=Path);a=p.parse_args();root=a.directory
m=json.loads((root/'summary.json').read_text());plant=json.loads((root/'plant.json').read_text())
def factors(roots):
    terms=[]
    for x,y in roots:
        if y < -1e-7:continue
        if abs(y)<1e-7:terms.append(f'(s {(-x):+.6g})')
        else:terms.append(f'(s² {(-2*x):+.6g} s {(x*x+y*y):+.6g})')
    return terms
rows=[]
for label,rr in [('D(s)',m['plant_poles']),('Nψ(s) / 1224.36945',m['transfer_functions']['heading_zeros']),('Np(s) / 2611.15459',m['transfer_functions']['roll_zeros'])]:
    rows.append('<tr><th>'+label+'</th><td>'+ '<br> × '.join(html.escape(x) for x in factors(rr))+'</td></tr>')
poles=np.array(m['selected_poles']);mode=poles[np.argmin(abs(poles[:,1]-15.96853))]
zeta=-mode[0]/np.linalg.norm(mode)
report='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Classical lateral control analysis</title><style>body{font:17px/1.55 system-ui;color:#17313d;max-width:1150px;margin:32px auto;padding:0 20px}img{width:100%}pre{background:#eef3f6;padding:18px;white-space:pre-wrap;overflow-wrap:anywhere}table{border-collapse:collapse}td,th{border-bottom:1px solid #ccd;padding:10px;text-align:left;vertical-align:top}code{font-size:.94em}</style>
<h1>Classical analysis of the current lateral controller</h1><p>10 October 2026 · straight level Trout Lake trim · native high-DOF mission model</p>
<p><strong>The selected gains are locally stable; the earlier gains are unstable in this analysis.</strong> The selected loop has a combined gain margin of 1.524× (3.66 dB), with the limiting crossing at 2.612 Hz. Increasing heading-rate P alone from 0.005 to approximately 0.007906 crosses a stability boundary with I and roll damping held fixed. These are properties of this local simulated plant, not flight limits.</p>
<h2>Operating point and linearization</h2><p>Total mass 1.758 kg; voltage 22.2 V; speed 4.383 m/s; throttle 44.59%; left brake 0 mm and right brake 3.747 mm. The normalized differential command is u₀=0.05205. The SERVO4 span is 900–2400 μs, mapped through the 55 mm drum to 71.995 mm per unit normalized command. Small perturbations stay on the right-brake branch. The zero-command mixer switch is not differentiable and is not used as the operating point.</p>
<p>The linearization retains all 14 internal mechanical states after removing irrelevant global position and heading. They are payload roll/pitch, three relative canopy-attitude coordinates, payload body velocity, and each body's angular velocity. Propeller RPM is held at its fixed-throttle equilibrium; all mechanical gyroscopic and torque terms remain in the plant. No aerodynamic coefficients or controller gains were changed.</p>
<pre>Δẋ = A Δx + B Δu
[Δψ̇, Δp]ᵀ = C Δx

Gψ(s) = Cψ (sI − A)⁻¹ B = Nψ(s) / D(s)
Gp(s) = Cp (sI − A)⁻¹ B = Np(s) / D(s)</pre>
<p>Outputs are in degrees/second; the input is normalized brake fraction. A symmetric finite-difference Jacobian of the sampled simulator is converted to a continuous realization through the augmented matrix logarithm. Halving both the numerical time step and perturbation changes the complex response by less than 0.00005% over 0.01–30 Hz. The trim step residual is below 4×10⁻¹⁶ in the internal-state chart.</p>
<h2>The actual plant transfer functions</h2><p>The following are factored numerator and denominator polynomials, rounded for readability. Multiply the factors within each row. The numerator constants are 1224.36945 for heading rate and 2611.15459 for body roll rate. Full-precision coefficients and state-space matrices are in <a href="plant.json">plant.json</a> and <a href="plant.npz">plant.npz</a>. State-space evaluation, rather than rounded polynomial arithmetic, generates the plots.</p><table>'''+''.join(rows)+'''</table>
<p>The static gains are Gψ(0)=98.00 deg/s per brake fraction and Gp(0)=−13.05 deg/s per fraction. A +0.01 change in command therefore gives approximately +0.98 deg/s of steady heading rate locally. With speed scale Sᵥ=5/4.383, inverse steady-gain feedforward is 1/[SᵥGψ(0)]=0.00894, close to the selected 0.009.</p>
<p>This is not well represented by one first-order lag over the useful modal range. Balanced reductions through order 12 failed to keep both complex transfer responses within 5% over 0.05–10 Hz; all results here retain the full plant. The heading-rate channel has a right-half-plane zero pair near +8.148 ± j24.766 s⁻¹. The roll-rate channel also has right-half-plane zeros. These are model-derived nonminimum-phase features, not flight-identified zeros.</p>
<figure><img src="plant-bode.png"><figcaption>Plant Bode plots. Orange points are nonlinear sine tests; lines are the continuous local plant. The slight high-frequency phase difference is the input zero-order hold.</figcaption></figure>
<h2>Control law</h2><p>Let r be shaped heading-rate demand, H(s)=25.133/(s+25.133) the 4 Hz measurement filter, and Sᵥ≈1.1408 the speed scale at trim. The two filters have the same transfer function but independent states. Write the unsaturated local controller as:</p>
<pre>e = r − H(s) ψ̇

u = Sᵥ KFF r + Sᵥ KP e + (KI/s)e − Sᵥ KRD H(s)p

KFF=0.009; KP=0.005; KI=0.002; KRD=0.003
r(s) ≈ [1/(0.1s+1)] r_requested(s)

L(s) = H(s) [(Sᵥ KP + KI/s)Gψ(s) + Sᵥ KRD Gp(s)]

ψ̇(s)/r_requested(s)
 = Gψ(s)[Sᵥ(KFF+KP)+KI/s] / [(1+L(s))(0.1s+1)]</pre>
<p>The full implementation also limits demand to ±25 deg/s, acceleration to 40 deg/s², output to ±1, and the integrator to ±0.15, with conditional integration. Those bounds are inactive in this local analysis. Heading-acceleration feedforward is zero. The speed-scaled terms use the same Sᵥ; the integral term does not. Feedforward and target shaping alter command response but do not move the feedback characteristic roots of 1+L(s).</p>
<p><strong>Why p is not φ̇:</strong> p=φ̇−ψ̇ sinθ. At fixed bank and pitch, a steady turn therefore has p=−ψ̇ sinθ even though φ̇=0. This trim has about 7.65° payload pitch, explaining Gp(0)/Gψ(0)≈−0.133. The roll-rate term damps motion but also provides a steady-turn contribution that the heading loop must accommodate.</p>
<h2>Loop Bode plot and root locus</h2><figure><img src="loop-bode.png"><figcaption>Return ratio with both feedback paths included. Earlier P=0.04 and RD=0.03 use the same I=0.002 and 4 Hz filter here. Their command feedforward settings are irrelevant to this pole comparison.</figcaption></figure>
<p>There are three unity-gain crossings: 0.0321, 5.773 and 5.980 Hz. The first has a 104° phase offset from −180°, but the later crossings occur after further phase winding. A single quoted phase margin would hide the loop geometry. The negative-real crossing at 2.612 Hz, plus direct closed-loop eigenvalues, gives the more useful result: a common feedback gain increase of 52.4% reaches instability.</p>
<figure><img src="root-locus.png"><figcaption>Left: multiply the entire feedback return ratio by λ, retaining its relative gains. Right: vary KP only while KI and KRD remain fixed. Green circles are the selected poles; red markers identify stability boundaries. Axes show the modes near the stability boundary; some fast branches leave the plot.</figcaption></figure>
<p>The selected controller's limiting oscillatory pair is approximately −0.986 ± j15.969 s⁻¹: 2.54 Hz with damping ratio 0.062. The slowest real pole is −0.158 s⁻¹. The earlier gains produce an unstable pair near +3.875 ± j17.537 s⁻¹ (2.79 Hz). This is consistent with the earlier native SITL excitation, without claiming to reproduce its nonlinear amplitude or every flight mode.</p>
<p>A 400 Hz sampled-data check uses the native backward-Euler measurement filters and same-tick integrator update. Its spectral radius is 0.999606 for the selected gains and 1.009699 for the earlier gains: the local stability conclusion survives discretization.</p>
<h2>Checks against the nonlinear simulator</h2><p>Five frequencies—0.1, 0.35, 1, 2.6 and 6 Hz—were tested at both ±0.0005 and ±0.001 brake fraction. With the input zero-order hold accounted for, the worst relative complex response error was below 0.005%. This verifies the local numerical linearization of this simulator, not the physical aircraft. The infinitesimal model excludes PWM quantization.</p>
<figure><img src="closed-step.png"><figcaption>A +0.2 deg/s command increment. The dashed trace runs the nonlinear plant, brake travel-rate limiter and sampled controller using truth gyro/attitude; it is not a native EKF/SITL run. The solid trace is the continuous linear prediction.</figcaption></figure>
<h2>Scope and next experiment</h2><p>Throttle is fixed, the navigation outer loop is outside the loop break, and sensing is ideal payload gyro/attitude. EKF dynamics, transport delay, PWM quantization and active TECS are not represented. The modeled servo travel limit is inactive for the infinitesimal plant, and is exercised as implemented in the nonlinear step check. These are small perturbations about one right-brake operating point, not a global model of both turn directions.</p>
<p>The next useful design experiment is to reduce the lightly damped 2.5 Hz feedback response while retaining the useful low-frequency turn tracking. Before choosing new gains, repeat this calculation at left/right turn trims and the softer-roll profile; test how delay and filtering shift the margin. A simple P increase is poorly motivated by this result.</p>
<h2>Reproduction</h2><pre>python3 research/archive/legacy-tools/classical_lateral.py --model /path/to/ardupilot/Tools/simulation/paraglider --output /path/to/research/classical-lateral
python3 research/archive/legacy-tools/report_classical_lateral.py /path/to/research/classical-lateral</pre><p>The scripts are in the tinkering repository. Raw matrices, traces and this research log remain in Mission Planner. Model source revision: '''+html.escape(m['model_source_revision'])+'''. Remove the local trim cache when changing model configuration. <a href="summary.json">All numerical results and validation</a>.</p><p>Method reference: <a href="https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.ss2tf.html">SciPy state-space to transfer-function conversion</a>.</p></html>'''
(root/'report.html').write_text(report)
print(root/'report.html')


# Modal annotations are generated independently by annotate_lateral_roots.py.
if (root/'annotated-root-locus.png').exists():
    target=root/'report.html'
    target.write_text(target.read_text()+'<h2>Annotated modes and zeros</h2><img src="annotated-root-locus.png"><img src="plant-zero-map.png">')