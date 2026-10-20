from pathlib import Path
import json

HERE = Path(__file__).resolve().parent
ap = json.loads((HERE/"ap-summary.json").read_text())
lines = ["# Campaign measurements", "", "## Actual ArduPlane LAND", "",
         "Incoming values are measured just above contact. All six finishes use classical control.", "",
         "| Approach | Spot error (m) | Along error (m) | Cross error (m) | Sink (m/s) | Ground speed (m/s) |",
         "|---|---:|---:|---:|---:|---:|"]
for r in ap:
    if r["kind"] == "landing":
        values = [r[k] for k in ["spot_error", "along_error", "cross_error",
                                "touchdown_sink", "touchdown_ground_speed"]]
        lines.append("| "+r["id"]+" | "+" | ".join(f"{x:.2f}" for x in values)+" |")
lines += ["", "## Equal pilot-request recovery", "",
          "Recovery time includes the initial eight-second descent, ten-second climb and five-second settling requirement.", "",
          "| Controller / turn request | Altitude loss (m) | Peak sink (m/s) | Peak pitch (deg) | Recovery time (s) |",
          "|---|---:|---:|---:|---:|"]
for r in ap:
    if r["id"].startswith("matched-input"):
        values = [r[k] for k in ["altitude_loss", "peak_sink", "peak_pitch_deg", "recovery_time"]]
        lines.append("| "+r["id"]+" | "+" | ".join("not settled" if x is None else f"{x:.2f}" for x in values)+" |")
lines += ["", "These four runs support further recovery work, not a statistical superiority claim.",
          "LQI remained active throughout these recovery maneuvers.", "",
          "## Native flare bias sensitivity", "",
          "1 m indicated-height trigger, 54 mm common brake. Sink perturbation is applied at 3 m true AGL.",
          "", "| Height bias (m) | Sink disturbance (m/s) | Incoming sink (m/s) | Ground speed (m/s) |",
          "|---:|---:|---:|---:|"]
for r in json.loads((HERE/"refine-summary.json").read_text()):
    if r["spec"]["id"].startswith("bias"):
        s, c = r["spec"], r["metrics"]["contact"]
        lines.append(f'| {s["height_bias"]:.2f} | {s["sink_perturbation"]:.2f} | {c["sink"]:.3f} | {c["ground_speed"]:.3f} |')
(HERE/"results.md").write_text("\n".join(lines)+"\n")
