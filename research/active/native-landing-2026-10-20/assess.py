"""Assess actual packaged firmware flight traces; truth is evaluation only."""
import hashlib, json, math, tarfile
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
rows=[]
for spec in json.loads((HERE/"cases.json").read_text()):
    path=HERE/"runs"/(spec["id"]+"-result.json")
    d=json.loads(path.read_text())
    contact=d["contact"]
    assert contact is not None, (spec["id"],d["failure"])
    incoming=contact["incoming"]; state=incoming["state"]; target=d["target"]
    airborne=[x for x in d["samples"] if x["time"]<=incoming["time"]]
    approach=[x for x in airborne if x["telemetry"].get("landing",{}).get("phase")=="approach"]
    flare=[x for x in airborne if x["telemetry"].get("landing",{}).get("phase")=="flare"]
    fraction=sum(x["active"] for x in approach)/max(1,len(approach))
    row={"id":spec["id"],"failure":d["failure"],"sink_m_s":state[9],
      "ground_speed_m_s":math.hypot(state[7],state[8]),
      "spot_error_m":math.hypot(state[0]-target["north"],state[1]-target["east"]),
      "approach_lqi_fraction":fraction,"modes":sorted(set(x["mode"] for x in airborne)),
      "flare_max_throttle":max((x["command"][0] for x in flare),default=None),
      "flare_onset_truth_height_m":-flare[0]["state"][2] if flare else None,
      "final_phase":d["finalLanding"]["phase"],"flight_seconds":incoming["time"]}
    assert row["modes"]==[10], row
    assert row["flare_max_throttle"]==0, row
    assert not any(x["active"] for x in flare), row
    assert fraction >= .95 if spec.get("scheme","lqi")=="lqi" else fraction==0, row
    assert d["failure"] is None and row["final_phase"]=="landed", row
    assert 0 < row["sink_m_s"] < 2, row
    assert row["spot_error_m"] < 25, row
    rows.append(row)
(HERE/"summary.json").write_text(json.dumps(rows,indent=2)+"\n")
lines=["# Native AUTO landing results", "", "Each case uses packaged ArduPlane WASM, its EKF, and the Rust 9-DOF model. No landing RC guidance runs in JavaScript. Truth is used only for setup and touchdown evaluation.","", "| Case | Sink at contact (m/s) | Ground speed (m/s) | Spot error (m) | LQI approach |", "|---|---:|---:|---:|---:|"]
for x in rows: lines.append(f"| {x['id']} | {x['sink_m_s']:.2f} | {x['ground_speed_m_s']:.2f} | {x['spot_error_m']:.2f} | {x['approach_lqi_fraction']:.1%} |")
lines += ["", "All cases stayed in AUTO; native final disengaged LQI, and brake flare had zero commanded throttle. Playback ended after fused near-ground/low-speed confirmation. This is not a statistical campaign, a flight qualification, or a validated ground-roll simulation.","", "The failed-lidar case uses the normal AP landing-height fallback. The simulator endpoint uses fused altitude above the known mission field when HAGL is unavailable. It does not assume zero AGL.","", "Initial 100%/s brake application at 1.05 m produced 1.71 m/s sink in the long mission. The selected simulator settings use 300%/s and 1.3 m, with unchanged LQI gains. These are airframe-specific experimental flare settings."]
(HERE/"RESULTS.md").write_text("\n".join(lines)+"\n")
with tarfile.open(HERE/"evidence.tar.gz","w:gz") as archive:
    specs=json.loads((HERE/"cases.json").read_text())
    if (HERE/"runs/release-smoke-result.json").exists():
        specs.append({"id":"release-smoke"})
    for spec in specs:
        path=HERE/"runs"/(spec["id"]+"-result.json")
        archive.add(path,arcname=path.name)
print(json.dumps(rows,indent=2))
