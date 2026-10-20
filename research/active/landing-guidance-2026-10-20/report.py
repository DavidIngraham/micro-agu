import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
rows=json.loads((HERE/"summary.json").read_text())
lines=["# Landing guidance measurements","",
       "One run per condition; incoming values immediately above the model contact surface.",
       "Stock uses an LQI-selected AUTO approach and classical LAND. Guided variants use LQI in CRUISE, then an external research flare through the manual mixer.","",
       "| Case | Finished | Spot error (m) | Sink (m/s) | Ground speed (m/s) | Pitch (deg) | LQI approach active |",
       "|---|---|---:|---:|---:|---:|---:|"]
for r in rows:
    metrics=[f'{r[k]:.2f}' if k in r else "—" for k in ["spot_error","sink","ground_speed","pitch"]]
    active=r["approach_active_fraction"]
    lines.append("| "+r["id"]+" | "+str(r["complete"])+" | "+" | ".join(metrics)+
                 " | "+(f"{active:.0%}" if active is not None else "—")+" |")
lines+=["","Failures: "+str([r for r in rows if not r["complete"]]),"",
        "These results separate position accuracy from severity; reaching the boundary is not a soft-landing pass."]
(HERE/"results.md").write_text("\n".join(lines)+"\n")
