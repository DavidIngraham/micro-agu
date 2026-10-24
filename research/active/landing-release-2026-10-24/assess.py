"""Assess matched native-runtime landings and retain unfavorable results."""
import hashlib
import json
import math
from pathlib import Path
import statistics

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


rows = []
for filename in ("cases.json", "confirmation-cases.json"):
    for spec in json.loads((HERE / filename).read_text(encoding="utf-8-sig")):
        path = HERE / "runs" / (spec["id"] + "-result.json")
        data = json.loads(path.read_text())
        assert data["spec"] == spec, path
        assert data["environment"]["windNED"] == spec["windNED"], spec
        variant = spec["id"].split("-", 1)[0]
        expected_aim = 0 if variant == "candidate" else 50
        assert abs(data["parameters"]["LAND_FLARE_AIM"] - expected_aim) < 1e-5
        incoming = data["contact"]["incoming"]
        q = incoming["state"]
        n, e = q[0] - data["target"]["north"], q[1] - data["target"]["east"]
        heading = 0 if spec.get("fixedMission") else data["target"]["heading"]
        rows.append({
            "id": spec["id"], "variant": variant, "condition": spec["name"],
            "confirmation": filename != "cases.json", "spec": spec,
            "complete": data["complete"], "failure": data["failure"],
            "final_phase": data["finalLanding"]["phase"],
            "spot_error_m": math.hypot(n, e),
            "along_error_m": n * math.cos(heading) + e * math.sin(heading),
            "cross_error_m": -n * math.sin(heading) + e * math.cos(heading),
            "sink_m_s": q[9], "ground_speed_m_s": math.hypot(q[7], q[8]),
            "modes": sorted(set(s["mode"] for s in data["samples"] if s["time"] <= incoming["time"])),
            "landing_parameters": {k: v for k, v in data["parameters"].items() if k.startswith(("LAND_", "PG_FLARE", "TECS_LAND"))},
            "result_sha256": digest(path),
        })


def statistics_for(variant):
    primary = [r for r in rows if r["variant"] == variant and not r["confirmation"]]
    all_runs = [r for r in rows if r["variant"] == variant]
    tails = [r for r in all_runs if r["condition"] == "tail"]
    return {
        "primary_cases": len(primary), "total_runs": len(all_runs),
        "completed": sum(r["complete"] for r in all_runs),
        "median_miss_m": statistics.median(r["spot_error_m"] for r in primary),
        "worst_miss_m": max(r["spot_error_m"] for r in primary),
        "rms_miss_m": math.sqrt(statistics.fmean(r["spot_error_m"] ** 2 for r in primary)),
        "median_sink_m_s": statistics.median(r["sink_m_s"] for r in primary),
        "worst_sink_m_s": max(r["sink_m_s"] for r in all_runs),
        "worst_ground_speed_m_s": max(r["ground_speed_m_s"] for r in all_runs),
        "tail_median_sink_m_s": statistics.median(r["sink_m_s"] for r in tails),
        "tail_sink_range_m_s": [min(r["sink_m_s"] for r in tails), max(r["sink_m_s"] for r in tails)],
    }


stats = {v: statistics_for(v) for v in ("published", "candidate")}
old, new = stats["published"], stats["candidate"]
checks = {
    "all_candidate_runs_complete": new["completed"] == new["total_runs"],
    "candidate_modes_auto_only": all(r["modes"] == [10] for r in rows if r["variant"] == "candidate"),
    "median_miss_reduced": new["median_miss_m"] < old["median_miss_m"],
    "worst_miss_reduced": new["worst_miss_m"] < old["worst_miss_m"],
    "raw_peak_sink_not_increased": new["worst_sink_m_s"] <= old["worst_sink_m_s"],
    "raw_peak_ground_speed_not_increased": new["worst_ground_speed_m_s"] <= old["worst_ground_speed_m_s"],
    "existing_native_regression_sink_limit": all(0 < r["sink_m_s"] < 2 for r in rows if r["variant"] == "candidate"),
}
result = {"rows": rows, "statistics": stats, "checks": checks}
(HERE / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
provenance = {
    "files": {p: digest(HERE / p) for p in ("case.mts", "run.py", "cases.json", "confirmation-cases.json", "plan.json", "configuration.json", "published-build.json", "assess.py")},
    "firmware": {str(p.relative_to(HERE)): digest(p) for p in (HERE / "firmware").rglob("*") if p.is_file()},
    "runtime_inputs": {p: digest(ROOT / p) for p in ("viewer/generated/physics/sim_wasm_bg.wasm", "aircraft/microagu/controllers/lqi-gains.bin")},
    "note": "Truth is evaluation only. Full local traces are ignored and bound by per-result hashes. Published endpoint failures retained.",
}
(HERE / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

conditions = [r["condition"] for r in rows if r["variant"] == "published" and not r["confirmation"]]
fig, axes = plt.subplots(1, 2, figsize=(12, 5), layout="constrained")
for variant, offset, color in [("published", -.18, "#b4561c"), ("candidate", .18, "#008366")]:
    selected = [next(r for r in rows if r["variant"] == variant and r["condition"] == c and not r["confirmation"]) for c in conditions]
    y = [j + offset for j in range(len(conditions))]
    axes[0].barh(y, [r["spot_error_m"] for r in selected], height=.34, label=variant, color=color)
    axes[1].barh(y, [r["sink_m_s"] for r in selected], height=.34, label=variant, color=color)
for ax in axes:
    ax.set_yticks(range(len(conditions)), conditions)
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=.2)
    ax.legend(fontsize=8)
axes[0].set(title="Contact distance from target", xlabel="Spot miss (m)")
axes[1].set(title="Downward speed before contact", xlabel="Sink (m/s)")
fig.suptitle("Landing calibration: 3 m/s wind-update gate and 0% flare aim", fontsize=14)
fig.savefig(HERE / "comparison.png", dpi=150)
print(json.dumps({"statistics": stats, "checks": checks}, indent=2))
