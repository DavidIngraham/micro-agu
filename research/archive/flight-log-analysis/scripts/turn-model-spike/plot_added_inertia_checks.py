from pathlib import Path
import numpy as np,json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
O=Path(__file__).parent;a=np.load(O/'added-body-conservation.npz')
fig,axes=plt.subplots(1,3,figsize=(14,4))
axes[0].plot(a['t'],a['energy_error']*1e6);axes[0].set_ylabel('Energy drift (microjoules)')
axes[1].plot(a['t'],np.linalg.norm(a['momentum_error'],axis=1)*1e6);axes[1].set_ylabel('Momentum residual (micro kg m/s)')
axes[2].plot(a['t'],a['hinge_gap']*1e6);axes[2].set_ylabel('Suspension joint gap (micrometres)')
for ax in axes:ax.set_xlabel('Time (s)');ax.grid(alpha=.2)
fig.suptitle('Added-inertia mechanics: gravity-only conservation test',fontsize=15);fig.tight_layout(rect=(0,0,1,.92));fig.savefig(O/'iteration-added-inertia-checks.png',dpi=150);plt.close(fig)
p=O/'physics-development.html';s=p.read_text(encoding='utf-8');section='''<section id="added-inertia"><h2>Longer records and a missing inertia term</h2><figure><img src="iteration-circle-wind.png" alt="Wind estimates from heading and velocity-circle assumptions across twenty-second records"><figcaption>Sixty-eight twenty-second turning segments with at least 120 degrees of heading change and throttle range below 18 percentage points. Circle fitting assumes constant horizontal airspeed; heading fitting assumes negligible sideslip. Both use the displayed segment and are diagnostics, not independent forecast inputs. Median circle-fit speed residual is 0.39 m/s. Neither method supplies a measured airspeed.</figcaption></figure><p>The next physical candidate adds apparent air inertia to the two-body model. <a href="https://jtam.pl/pdf-102141-33701?filename=Apparent-masses-and-inert.pdf">Kowaleczko (2014)</a> describes directional apparent masses, distinct effective centers, and translation/rotation coupling for parafoils. Our implementation uses an energy-based spatial inertia formulation. Its coefficients and effective centers remain provisional; they have not been measured for this canopy.</p><pre>ν = [v_air,body; ω_body]
H = H_rigid + A_added
[P; H_angular] = H ν
H ν̇ = [F − ω × P;
        M − ω × H_angular − v_air,body × P]

A_added = Σ B_iᵀ A_i B_i + diag(0, J_added)
B_i = [I, −skew(r_i)]
T_fluid = ½ νᵀ A_added ν</pre><p>Positive directional masses and rotational inertias construct a symmetric positive added-inertia matrix. Offset centers create the coupling blocks automatically. Suspension forces enter both bodies with equal and opposite force and the appropriate moment arms; the joint constraint is solved using the full spatial inverse inertia. Uniform steady wind is assumed. Real mass alone supplies weight; apparent inertia does not add gravitational load.</p><figure><img src="iteration-added-inertia-checks.png" alt="Energy, momentum and suspension-constraint residuals for the added-inertia solver"><figcaption>Five-second gravity-only test at 1.25 ms integration steps, with aerodynamic forces, propulsion and joint springs/dampers disabled. This verifies the mechanics, not the flight fit. Energy drift is 5.3 microjoules; joint gap stays below 0.7 micrometres. Halving the step from 2.5 ms reduced energy error by about 16×.</figcaption></figure><p>Forty randomized instantaneous checks give maximum zero-added-inertia parity error of 2.4×10⁻¹³ against the earlier solver and energy-rate residual below 9×10⁻⁹ W. Linear and angular momentum balances also pass. Source: <code>fast_added_body.cpp</code>, <code>test_added_body.py</code>; evidence: <code>added-body-mechanics-checks.json</code>. The initial flight experiment fits a single shared scale on a provisional added-inertia tensor along with the existing aerodynamic coefficients; no production simulator files are changed.</p></section>'''
if 'id="added-inertia"' not in s:p.write_text(s.replace('</html>',section+'</html>'),encoding='utf-8')
