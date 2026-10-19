import json,hashlib,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from run import OUT,BIN
r=json.loads((OUT/"results.json").read_text())
c=json.loads((OUT/"checks.json").read_text())
constant=json.loads((OUT/"constant-results.json").read_text())
labels=["baseline","f2.5-d0.7-a-0.005-b0.0","f1.6-d0.5-a-0.08-b0.0","f1.0-d0.15-a-0.03-b0.2"]
names=["Baseline","Weak twist","Lowest composite score","Largest yaw amplitude"]
summary={}
for flight in ["hood_river","trout_lake"]:
 summary[flight]={}
 for v in labels:
  a=[x for x in r if x["flight"]==flight and x["variant"]==v and "error" not in x]
  summary[flight][v]={k:np.mean([x["metrics"][k] for x in a],axis=0).tolist() for k in ["uncentered_rmse","model_band_rms","observed_band_rms","score","model_peak_hz"]}
  summary[flight][v]["phase_abs_error"]=float(np.mean([abs(x["metrics"]["phase_error_deg"]) for x in a]))
fig,axes=plt.subplots(1,2,figsize=(11,4),layout="constrained")
for ax,flight in zip(axes,summary):
 base=summary[flight]["baseline"]
 for v in sorted(set(x["variant"] for x in r)):
  a=[x for x in r if x["flight"]==flight and x["variant"]==v]
  if any("error" in x for x in a):continue
  rmse=np.mean([x["metrics"]["uncentered_rmse"][1] for x in a])
  amp=np.mean([x["metrics"]["model_band_rms"][2] for x in a])
  ax.scatter(rmse/base["uncentered_rmse"][1],amp/base["observed_band_rms"][2],s=18,color="#a3acb4",alpha=.6)
 for v,name,color in zip(labels,names,["black","#2c8c70","#d18825","#b64f79"]):
  x=summary[flight][v]
  ax.scatter(x["uncentered_rmse"][1]/base["uncentered_rmse"][1],x["model_band_rms"][2]/base["observed_band_rms"][2],s=60,label=name,color=color)
 ax.axhline(1,color="gray",ls="--");ax.axvline(1,color="gray",ls=":")
 ax.set(title=flight.replace("_"," ").title(),xlabel="Pitch-rate RMSE / baseline",ylabel="Yaw-band RMS / observed")
axes[0].legend(fontsize=8)
fig.savefig(OUT/"tradeoff.png",dpi=180);plt.close(fig)
lines=["# Minimal asymmetric canopy-twist experiment","",
"AI-assisted research, 2026-10-18. No maintained simulator physics, ArduPilot gains, browser UI or deployment changed.","",
"## Result","",
"The extension can create growing coupled motion, but no tested case is accepted as an explanation of the observed flight response. Weak, stable coupling barely changes the missing oscillation. Strong coupling can reproduce Trout Lake's band amplitude while degrading instantaneous pitch/yaw prediction and failing to reproduce Hood River. The lowest composite score is therefore rejected.","",
'## Model','','One deformation coordinate and its rate are added in this research-only Rust executable. Each panel twists in proportion to its normalized span coordinate. Incidence, force direction, center-of-pressure position and local velocity change with twist, including motion-induced local airflow. The undeformed load is subtracted before applying modified loads through the existing external-wrench interface.','','The mode coordinate $\\eta$ follows','','$$','J_\\eta\\ddot\\eta+2\\zeta\\omega J_\\eta\\dot\\eta+J_\\eta\\omega^2(\\eta+\\eta^3/\\eta_s^2)=Q_{\\rm panels}+Q_\\beta.','$$','','Here $J_\\eta=0.001$ kg m², $\\eta_s=0.35$ rad, $\\omega=2\\pi f$, and $Q_{\\rm panels}=\\sum_i(\\partial\\mathbf r_i/\\partial\\eta)\\cdot\\mathbf F_i$. The optional phenomenological side-load term is $Q_\\beta=J_\\eta\\omega^2g_\\beta\\beta(V/4)^2$. Panel pitch is $s_i\\eta$, with $s_i=-0.8,-0.4,0,0.4,0.8$. Reported twist angles are modal coordinates; outer modeled panels see 0.8 times that angle.','','Stiffness and damping are positive; the cubic spring stiffens the response. There is no prescribed oscillation or negative structural damping. Panel aerodynamic feedback can nevertheless create instability. The generalized panel force matches virtual work for the chosen center-of-pressure displacement.','','Elastic-axis-to-center-of-pressure offset has both signs in the sweep. These offsets, modal inertia, unloaded frequency, damping, side-load coefficient and mode shape are unmeasured hypotheses. This is a reduced-order aerodynamic feedback experiment, not a fully energy-consistent flexible multibody implementation: rigid/modal inertial cross terms, deformation changes to added mass, fabric inflation/pressure and sectional aerodynamic pitching moments in the deformation equation are omitted. The rigid canopy pitching-moment law remains active. Instability boundaries must not be treated as physical flutter predictions.','',
"## Protocol","",
"89 variants × ten 30-second historical windows = 890 replays. The first grid contains baseline plus 72 combinations: unloaded frequency 1, 1.6, 2.5 Hz; damping ratio 0.15 or 0.5; signed offset/chord -0.08, -0.03, 0.03, 0.08; side-load gain -0.2, 0, 0.2. A follow-up weak-coupling grid adds 16 cases: 2.5 or 4 Hz, damping 0.3 or 0.7, offsets/chord ±0.005 or ±0.01, zero side-load gain.",
"",
"Every variant uses shared canopy parameters for both flights, their as-built payload geometries and 132 mm riser spacing, existing initial canopy-attitude inference and measured actuator commands. Initial modal twist/rate are zero. No hidden forcing is fitted to the measured gyro. All windows were previously studied; there is no independent held-out flight or parameter-identification claim.",
"",
"Sixty replays hit the invalid/>1 rad twist guard and are retained as failed, not silently excluded from counts. The other 830 completed. Completed cases with large deformation are still not automatically credible.",
"",
"## Flight response","",
"Window-mean uncentered body-rate RMSE and 1.2–1.9 Hz yaw RMS, in deg/s:",
"",
"| Flight / case | Observed yaw-band RMS | Predicted yaw-band RMS | Pitch RMSE | Yaw RMSE |",
"|---|---:|---:|---:|---:|"]
for flight in summary:
 for v,name in zip(labels,names):
  x=summary[flight][v]
  lines.append(f"| {flight} / {name} | {x['observed_band_rms'][2]:.2f} | {x['model_band_rms'][2]:.2f} | {x['uncentered_rmse'][1]:.2f} | {x['uncentered_rmse'][2]:.2f} |")
lines+=["",
"The largest-amplitude case is f1.0-d0.15-a-0.03-b0.2. It generates 17.60 deg/s yaw-band RMS in Trout Lake versus 16.66 observed, but only 4.23 versus 26.80 in Hood River. Pitch RMSE worsens from 17.75 to 44.33 in Trout Lake and 26.79 to 51.09 in Hood River. Yaw errors also increase substantially.",
"",
"Its mean pitch/yaw phase error improves from 92° to 40° in Trout Lake, but worsens from 87° to 115° in Hood River. It is not a transferable match of the coupling. The lowest-composite-score case improves that score approximately 20% across the ten windows while substantially degrading raw-rate errors; amplitude-sensitive aggregate scoring is not an acceptance gate.",
"",
"![Amplitude versus pitch error](tradeoff.png)",
"",
"## Equilibrium and stability","",
"Six diagnostic variants were evaluated in both configurations, using 12 mm nominal common effective brake. The differential search spans almost the full both-brakes-engaged range; two positive-offset cases fail equilibrium convergence and receive no modal/decay interpretation. Ten equilibria meet maximum residual 1e-4.",
"",
"The finite-difference modal chart contains 15 rigid-body coordinates plus twist and twist rate. RPM perturbations are excluded; the nonlinear fixed-command simulations allow causal RPM evolution. Neutral heading and disabled baseline twist coordinates are not treated as physical instability.",
"",
"| Flight / case | Largest real eigenvalue (1/s) | Associated frequency (Hz) |",
"|---|---:|---:|"]
for x in constant:
 if x["variant"] not in labels or "eigenvalues" not in x:continue
 e=max(x["eigenvalues"],key=lambda z:z["real"])
 lines.append(f"| {x['flight']} / {names[labels.index(x['variant'])]} | {e['real']:.3f} | {abs(e['hz']):.3f} |")
lines+=["",
"The weak case is locally stable apart from neutral heading and decays after 1°, 5° and 15° modal perturbations and a 5° canopy-pitch perturbation. The lowest-score strong case has nonoscillatory divergence in both configurations; it should not be described as a matched self-sustaining oscillation. The largest-amplitude case has a growing oscillatory mode near 1.10 Hz in Hood River and 1.24 Hz in Trout Lake, along with other unstable behavior. Those are not a demonstrated match to the pervasive ~1.6 Hz component. They also do not establish an explanation for the separately observed 1.16 Hz CRUISE episode.",
"",
"Forty-second nonlinear perturbation checks are retained in constant-results.json. Late nonzero displacement can include drift toward a different equilibrium or turning condition; it does not by itself establish a limit cycle.",
"",
"## Verification","",
"Five Rust tests pass: zero-twist panel parity, no symmetric-flow twist forcing, structural restoring/damping signs, mirrored lateral response, and generalized-load virtual work. Disabled-extension logged replay matches the installed native baseline to 1.3e-12 in state coordinates.",
"",
f"160 numerical/initialization checks completed with no failures. Across the six diagnostic variants and ten windows, doubling integration resolution from 32 to 64 substeps changes body rates by at most {max(x['max_rate_difference'] for x in c if x['substeps']==64):.6f} deg/s.",
"",
"Initial twist ±5° changes the largest-amplitude case's mean yaw-band RMS from 12.25 to approximately 11.62–12.71 deg/s across both flights. The weak case stays near 2.66 deg/s. This sensitivity does not rescue the cross-configuration mismatch.",
"",
"## Decision","",
"No promotion or LQI retune. This experiment demonstrates that an added deformation feedback path can produce instability and larger motion, but does not identify the real aircraft mechanism. The assumed twist shape and force path are not supported strongly enough to justify more unrestricted coefficient fitting. Video review should discriminate antisymmetric twist, symmetric camber/breathing, tip motion and payload-relative motion before selecting a more elaborate deformation model.",
"",
"## Reproduce","",
"Build/test this standalone crate with cargo test --release and cargo build --release. Run run.py, constant.py and checks.py with control/.venv/bin/python; run report.py with research/.venv/bin/python. run.py recomputes the full grid on each invocation. selected-diagnostics.json records the diagnostic selection, not accepted parameter estimates. Dense traces and the binary are ignored; summaries, failures and input hashes are retained."]
(OUT/"README.md").write_text(chr(10).join(lines)+chr(10))
(OUT/"summary.json").write_text(json.dumps(summary,indent=2))
(OUT/"manifest.json").write_text(json.dumps(dict(parent_commit=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(),source_hashes={str(x.relative_to(OUT)):hashlib.sha256(x.read_bytes()).hexdigest() for x in list(OUT.glob("*.py"))+list((OUT/"src").glob("*.rs"))},binary_sha256=hashlib.sha256(BIN.read_bytes()).hexdigest(),replays=len(r),failed=sum("error" in x for x in r),decision="No qualified physics candidate; no deployment",build_note="Rust formatting and an additional virtual-work test were applied after the replay sweep; production calculation unchanged."),indent=2))
