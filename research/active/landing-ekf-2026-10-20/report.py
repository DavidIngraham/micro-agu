from pathlib import Path
import json
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/"summary.json").read_text())
lines=["# EKF landing results", "", "Existing LQI gains, no pitot or canopy truth. Metrics are sampled before physical ground contact, not after a collision impulse.", "", "| Scenario | Outcome | Spot error (m) | Incoming sink (m/s) | Ground speed (m/s) | Actual flare height (m) |", "|---|---|---:|---:|---:|---:|"]
for r in rows:
    metric=lambda key:f"{r[key]:.2f}" if r.get(key) is not None else "—"
    lines.append(f"| {r['id']} | {r['final_phase']} | {metric('spot_error')} | {metric('sink')} | {metric('ground_speed')} | {metric('flare_height')} |")
lines += ["", "Successful cases ended with estimated touchdown and a paused simulator. The fully failed sensor must be rejected before final. Any dropout abort is retained, not omitted.", "", "The old raw-range calm feedback trial reached about 1.35 m/s sink. These checks demonstrate the EKF-based path and current worker integration; they are not a statistical head-to-head improvement claim. The truth-height experiment's roughly 0.81 m/s is an ideal-sensing reference, not a hardware guarantee.", "", "The sensor's 50 ms modeled latency still exceeds EKF3's 25 ms range timing assumption. Full canopy collapse, ground rollout and real-flight flare performance are not qualified. See README.md for estimator validity, cancellation, final handoff and endpoint definitions."]
(HERE/"results.md").write_text("\n".join(lines)+"\n")
