import json,hashlib,subprocess
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from run import OUT
r=json.loads((OUT/"results.json").read_text())
modal=json.loads((OUT/"modal-results.json").read_text())
checks=json.loads((OUT/"checks.json").read_text())
labels=["baseline","J4.0-C0.25-D0.1","coupled-offset0.0-D0.1"]
summary={}
for f in ["hood_river","trout_lake"]:
    summary[f]={}
    for v in sorted(set(x["variant"] for x in r)):
        a=[x["metrics"] for x in r if x["variant"]==v and x["flight"]==f]
        summary[f][v]={k:np.mean([x[k] for x in a],axis=0).tolist() for k in ["uncentered_rmse","model_band_rms","observed_band_rms","score"]}
        summary[f][v]["phase_abs_error"]=float(np.mean([abs(x["phase_error_deg"]) for x in a]))
(OUT/"summary.json").write_text(json.dumps(summary,indent=2))
fig,axes=plt.subplots(1,2,figsize=(10,4),layout="constrained")
for ax,f in zip(axes,summary):
    b=summary[f]["baseline"]
    for v in summary[f]:
        x=summary[f][v]
        ax.scatter(x["uncentered_rmse"][1]/b["uncentered_rmse"][1],x["model_band_rms"][2]/x["observed_band_rms"][2],color="#8898aa",s=20)
    for v,color,name in zip(labels,["black","#db6d36","#357bb7"],["Baseline","Pitch redistribution","Plus roll-coupled offset"]):
        x=summary[f][v]
        ax.scatter(x["uncentered_rmse"][1]/b["uncentered_rmse"][1],x["model_band_rms"][2]/x["observed_band_rms"][2],color=color,s=65,label=name)
    ax.axhline(1,color="green",ls="--",label="Observed amplitude")
    ax.axvline(1,color="gray",ls=":")
    ax.set(title=f.replace("_"," ").title(),xlabel="Pitch-rate RMSE / baseline",ylabel="Yaw oscillation amplitude / observed",ylim=(0,1.05))
axes[0].legend(fontsize=8)
fig.savefig(OUT/"tradeoff.png",dpi=180);plt.close(fig)
lines=["# Pitch inertia and damping distribution study","",
"AI-assisted diagnostic study, 2026-10-18. No selected physics, controller gains, firmware or browser deployment changed.","",
"## Result","",
"Redistributing pitch inertia and damping within the tested bounds does not explain the missing coupled oscillation. Lower damping modestly increases oscillation amplitude but worsens pitch-rate error. The lowest composite score is not a satisfactory physical model.","",
"## Protocol","",
"450 logged-input replays: 45 variants across four Hood River and six Trout Lake windows, all 30 seconds. These are reused historical data, not independent validation flights. Logged actuator commands are replayed open loop with causal motor dynamics, fixed per-window wind and inferred initial canopy attitude. The session role labels are inherited; this sweep uses every window diagnostically and makes no held-out claim.","",
"Main factorial: added pitch inertia multipliers 0.25, 1, 4; aerodynamic pitch damping multipliers 0.25, 1, 4; joint pitch damping multipliers 0.1, 1, 3. Payload geometry/inertia and yaw damping are fixed in this factorial. Multipliers are broad sensitivity choices, not measured confidence limits.","",
"Added pitch inertia is multiplied before the existing 0.52624 global added-inertia scale. Structural canopy pitch inertia remains 0.008 kg m². The direct total pitch inertia therefore spans approximately 0.00866–0.01852 kg m², versus 0.01063 nominal. This does not sweep structural inertia independently or below its fixed contribution.","",
"Separate location checks vary added_z1 over -0.5, 0.25, 0.5 m and added_z2 over 0, 0.25, 0.75 m, alone and with the low-damping combination. In the actual mass matrix, added_z1 shifts the fore-aft added mass and directly changes pitch coupling. added_z2 primarily changes roll coupling; it is included as a cross-axis control, not mislabeled as pitch inertia. Translation–rotation coupling is recomputed by Rust.","",
"Payload pitch inertia uncertainty is tested separately at 0.7 and 1.3 times nominal with a congruence transformation of the inertia tensor, preserving positive definiteness. These are sensitivity bounds, not measured uncertainty.","",
"## Logged-input results","",
"Values below are means over windows within each flight. Band amplitude is yaw-rate RMS over 1.2–1.9 Hz. RMSE is uncentered, excluding the initial two seconds and final 0.4 seconds.","",
"| Flight / variant | Observed band RMS | Model band RMS | Pitch-rate RMSE | Yaw-rate RMSE |",
"|---|---:|---:|---:|---:|"]
for f in summary:
    for v in labels:
        x=summary[f][v]
        lines.append(f"| {f} / {v} | {x['observed_band_rms'][2]:.2f} | {x['model_band_rms'][2]:.2f} | {x['uncentered_rmse'][1]:.2f} | {x['uncentered_rmse'][2]:.2f} |")
lines+=["","All rates are deg/s. J4.0-C0.25-D0.1 means four times added pitch inertia, one quarter aerodynamic pitch damping, and one tenth joint pitch damping. The third variant additionally removes the roll-coupled added-mass offset.","","![Amplitude versus instantaneous pitch error](tradeoff.png)","","## Equilibrium, modes, and decay","",
"26 equilibria were solved across both configurations at 12 mm common effective brake; all residuals were below 1e-4. This deliberately keeps both brakes engaged. It is not a sustained-turn equilibrium or a low-brake slack test.",
"",
"Modes use a 15-state finite-difference map with RPM perturbations excluded. The nominal motor is equilibrated; RPM can evolve during the nonlinear decay runs. The 12 mm trim differs from earlier 6 mm mode studies. Neutral heading eigenvalues near zero are not diagnosed as instability.",
"",
"| Flight / variant | Nearby mode Hz | Damping ratio | Pitch/yaw amplitude ratio | Pitch relative to yaw phase |",
"|---|---:|---:|---:|---:|"]
for x in modal:
    if x.get("dt")!=.004 or x["variant"] not in labels:continue
    m=min(x["modes"],key=lambda z:abs(z["hz"]-1.6))
    lines.append(f"| {x['flight']} / {x['variant']} | {m['hz']:.3f} | {m['damping']:.3f} | {m['pitch_yaw_ratio']:.2f} | {m['phase']:.1f}° |")
decays=[d for x in modal for d in x.get("decay",[]) if d["perturb_deg"]>0]
lines+=["",
"The pitch-only redistribution mainly reduces damping of a faster mode near 3 Hz. It leaves the near-1.6 Hz mode strongly damped and with a substantially different shape from the observed Trout Lake pitch/yaw ratio around 0.37–0.47 and phase around -76 to -96 degrees. A free-mode shape is not itself a prediction of forced flight response; the script also retains differential-brake forced response at 1.5625 Hz.",
"",
f"All {len(decays)} nonzero 1°, 5°, and 15° canopy-pitch perturbations decayed in 40-second fixed-command runs. Maximum late/early relative-angle RMS ratio: {max(d['ratio'] for d in decays):.6g}. Early RMS uses 1–10 s and late RMS uses the last 10 s. The zero-perturbation controls remain at equilibrium. This does not rule out sustained forcing or a limit cycle elsewhere in the flight envelope.",
"",
"## Numerical and initialization checks","",
f"Baseline reproduces the maintained replay and the identity factorial reproduces baseline exactly. For baseline and the two illustrated redistribution cases, 32-to-64-substep replay checks have a maximum body-rate difference of {max(x['max_rate_difference'] for x in checks if x['substeps']==64):.6f} deg/s across ten windows.",
"",
"Initial canopy angle was also varied from 0.35 rad to 0.25 and 0.45 rad in all ten windows for those three cases. The mean predicted yaw-band RMS stays near 3.01–3.05 deg/s for the redistribution cases versus 20.72 deg/s observed across all windows; changing initialization does not close the gap. Full metrics are retained in checks.json. Baseline and pitch-redistributed modal calculations were repeated at 2 ms versus 4 ms; both retain the same qualitative mode and stability result.",
"",
"## Interpretation and next step","",
"This rejects these parameter changes as a sufficient explanation, not the possibility that physical inertias or damping are wrong. Joint yaw damping, aerodynamic yaw derivatives, forcing, motor/servo load effects and unobserved canopy motion remain confounders. The small benefit from a roll-coupled offset reinforces that fitting pitch in isolation is insufficient.",
"",
"Do not promote the lowest-score case or retune the controller against it. Next, use the measured pitch/yaw mode shape to constrain a joint yaw-damping/gyro/canopy-inertia study, and compare plausible excitation mechanisms. Video evidence of relative motion would help discriminate the alternatives; no new data are required to reproduce this study.",
"",
"## Reproduce","",
"From the repository root, run control/.venv/bin/python with run.py, modal.py and checks.py in this directory; use research/.venv/bin/python for report.py (Matplotlib). Raw replay traces are local ignored artifacts; protocol, input hashes, numerical results and this report are retained. No ArduPilot campaign is warranted because no physics candidate passes the dynamics comparison."]
(OUT/"README.md").write_text(chr(10).join(lines)+chr(10))
manifest=dict(parent_commit=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(),source_hashes={str(p.relative_to(OUT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.glob("*.py")},decision="No model selection or deployment; missing oscillation unresolved",replays=len(r),errors=sum("error" in x for x in r))
(OUT/"manifest.json").write_text(json.dumps(manifest,indent=2))
