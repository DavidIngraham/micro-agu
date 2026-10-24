"""Compare onboard wind telemetry with simultaneous atmosphere truth."""
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def mean(values):
    return statistics.fmean(values)


def rms(values):
    return math.sqrt(mean([v * v for v in values]))


def norm2(vector):
    return math.hypot(vector[0], vector[1])


rows = []
traces = {}
for spec in json.loads((HERE / "threshold-cases.json").read_text(encoding="utf-8-sig")):
    path = HERE / "runs" / (spec["id"] + "-result.json")
    data = json.loads(path.read_text())
    assert data["spec"] == spec, spec
    assert data["contact"] is not None, (spec, data["failure"])
    if "windNED" in spec:
        assert data["environment"]["windNED"] == spec["windNED"], spec
    contact = data["contact"]["incoming"]
    final = next(s for s in data["samples"] if s["telemetry"].get("landing", {}).get("phase") == "final")
    approach = [s for s in data["samples"] if final["time"] - 20 <= s["time"] < final["time"]]
    assert len(approach) > 300
    assert all(s["wind"]["estimate"] is not None for s in approach)
    truth = [s["wind"]["truth"] for s in approach]
    estimate = [s["wind"]["estimate"]["ned"] for s in approach]
    errors = [[e[i] - t[i] for i in range(3)] for t, e in zip(truth, estimate)]
    gs_errors = [s["telemetry"]["speed"] - norm2(s["state"][7:9]) for s in approach]
    mean_true = [mean([t[i] for t in truth]) for i in range(3)]
    mean_est = [mean([e[i] for e in estimate]) for i in range(3)]
    forward_air = []
    for sample in approach:
        nav = sample["telemetry"]
        wind = sample["wind"]["estimate"]["ned"]
        vn = nav["speed"] * math.cos(nav["course"]) - wind[0]
        ve = nav["speed"] * math.sin(nav["course"]) - wind[1]
        forward_air.append(math.cos(nav["pitch"]) * (
            math.cos(nav["yaw"]) * vn + math.sin(nav["yaw"]) * ve
        ) + math.sin(nav["pitch"]) * nav["climb"])
    n, e = contact["state"][0], contact["state"][1]
    row = {
        "id": spec["id"], "spec": spec, "window_s": 20,
        "failure": data["failure"], "final_phase": data["finalLanding"]["phase"],
        "modes": sorted(set(s["mode"] for s in data["samples"] if s["time"] <= contact["time"])),
        "truth_mean_ned_m_s": mean_true, "estimate_mean_ned_m_s": mean_est,
        "wind_bias_ned_m_s": [mean([v[i] for v in errors]) for i in range(3)],
        "estimate_span_ned_m_s": [max(e[i] for e in estimate) - min(e[i] for e in estimate) for i in range(3)],
        "reconstructed_forward_airspeed_range_m_s": [min(forward_air), max(forward_air)],
        "horizontal_wind_rmse_m_s": rms([norm2(v) for v in errors]),
        "north_wind_rmse_m_s": rms([v[0] for v in errors]),
        "east_wind_rmse_m_s": rms([v[1] for v in errors]),
        "vertical_wind_rmse_m_s": rms([v[2] for v in errors]),
        "ground_speed_rmse_m_s": rms(gs_errors),
        "max_ground_speed_error_m_s": max(abs(v) for v in gs_errors),
        "approach_lqi_fraction": mean([float(s["active"]) for s in approach]),
        "along_contact_error_m": n, "cross_contact_error_m": e,
        "contact_sink_m_s": contact["state"][9],
        "contact_ground_speed_m_s": norm2(contact["state"][7:9]),
        "brake_flare_observed_before_contact": any(s["telemetry"].get("landing", {}).get("phase") == "flare" for s in data["samples"] if s["time"] <= contact["time"]),
        "seconds_from_final_to_contact": contact["time"] - final["time"],
        "remaining_travel_m": norm2([n - final["state"][0], e - final["state"][1]]),
        "final_ground_speed_m_s": norm2(final["state"][7:9]),
        "parameters": {k: v for k, v in data["parameters"].items() if k.startswith(("ARSPD", "AHRS", "EK3"))},
        "result_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
    rows.append(row)
    traces[spec["id"]] = (data["samples"], final["time"])

(HERE / "threshold-summary.json").write_text(json.dumps(rows, indent=2) + "\n")
manifest = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
provenance = {
    "firmware_revision": manifest["revision"],
    "files": {p: hashlib.sha256((HERE / p).read_bytes()).hexdigest() for p in ["case-threshold.mts", "run-threshold.py", "threshold-cases.json", "assess-threshold.py", "THRESHOLD.md"]},
    "note": "Fresh paired 5 m/s and 3 m/s gate runs; only FuseSideslip forward-speed gate changes. WIND has no validity bit; LQI activity independently requires wind_valid.",
}
firmware_root = ROOT.parent / "ardupilot-lqi-pipeline"
source_path = "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp"
baseline_source = subprocess.check_output(["git", "show", f"{manifest['revision']}:{source_path}"], cwd=firmware_root)
changed_source = (firmware_root / source_path).read_bytes()
expected_source = baseline_source.replace(b"if (vel_rel_wind.x > 5.0f)", b"if (vel_rel_wind.x > 3.0f)")
assert baseline_source != expected_source and changed_source == expected_source
provenance["threshold_change"] = {
    "baseline_m_s": 5, "test_m_s": 3,
    "source_path": source_path,
    "base_source_sha256": hashlib.sha256(baseline_source).hexdigest(),
    "test_source_sha256": hashlib.sha256(changed_source).hexdigest(),
    "patch_sha256": hashlib.sha256((HERE / "threshold-only.patch").read_bytes()).hexdigest(),
    "git_diff": subprocess.check_output(["git", "diff", "--", source_path], cwd=firmware_root, text=True),
}
provenance["driver_versions"] = {
    "case-threshold-initial.mts": {
        "sha256": hashlib.sha256((HERE / "case-threshold-initial.mts").read_bytes()).hexdigest(),
        "primary_cases": [r["id"] for r in rows if r["id"].endswith(("-calm", "-weather"))],
        "note": "Calm and default weather remain valid; preliminary course-relative steady-wind comparisons are retained in threshold-initial-* files but excluded from primary results.",
    },
    "case-threshold.mts": {
        "sha256": hashlib.sha256((HERE / "case-threshold.mts").read_bytes()).hexdigest(),
        "primary_cases": [r["id"] for r in rows if r["id"].startswith("threshold-earth-")],
        "note": "Steady winds fixed in earth NED axes, identical per condition across both firmware versions.",
    },
}
provenance["test_firmware_sha256"] = {
    str(path.relative_to(HERE)): hashlib.sha256(path.read_bytes()).hexdigest()
    for path in (HERE / "threshold-firmware").rglob("*") if path.is_file()
}
for name in ("arduplane.js", "arduplane.wasm"):
    old = (HERE / "threshold-firmware/5m" / name).read_bytes()
    new = (HERE / "threshold-firmware/3m" / name).read_bytes()
    assert hashlib.sha256(old).hexdigest() == manifest["sha256"][name]
    assert new == (firmware_root / "build/wasm/bin" / name).read_bytes()
source_paths = [
    "libraries/AP_Landing/AP_Landing_Slope.cpp", "ArduPlane/GCS_MAVLink_Plane.cpp",
    "ArduPlane/Plane.cpp", "libraries/AP_AHRS/AP_AHRS_NavEKF3.cpp",
    "libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp",
    "libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp",
    "libraries/AP_NavEKF3/AP_NavEKF3_AirDataFusion.cpp",
    "libraries/AP_CustomControl/AP_CustomControl_LQI.cpp", "libraries/SITL/SIM_JSON.cpp",
]
provenance["pinned_source_sha256"] = {
    path: hashlib.sha256(subprocess.check_output(
        ["git", "show", f"{manifest['revision']}:{path}"],
        cwd=ROOT.parent / "ardupilot-browser-9dof")).hexdigest()
    for path in source_paths
}
for name, relative in {
    "arduplane.wasm": "viewer/generated/ardupilot/arduplane.wasm",
    "sim_wasm_bg.wasm": "viewer/generated/physics/sim_wasm_bg.wasm",
    "highdof.json": "aircraft/microagu/configurations/browser-physics.json",
    "lqi-gains.bin": "aircraft/microagu/controllers/lqi-gains.bin",
}.items():
    value = hashlib.sha256((ROOT / relative).read_bytes()).hexdigest()
    assert value == manifest["sha256"][name], name
    provenance.setdefault("runtime_sha256", {})[name] = value
(HERE / "threshold-provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

conditions = ["calm", "weather", "head", "cross", "tail"]
fig, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
for gate, offset, color in [(5, -.18, "#b4561c"), (3, .18, "#008366")]:
    selected = [next(r for r in rows if r["id"] == (f"threshold-{gate}m-{c}" if c in ("calm", "weather") else f"threshold-earth-{gate}m-{c}")) for c in conditions]
    ys = [j + offset for j in range(len(conditions))]
    axes[0].barh(ys, [r["horizontal_wind_rmse_m_s"] for r in selected], height=.34, label=f"{gate} m/s gate", color=color)
    axes[1].barh(ys, [r["along_contact_error_m"] for r in selected], height=.34, label=f"{gate} m/s gate", color=color)
for ax in axes:
    ax.set_yticks(range(len(conditions)), conditions)
    ax.invert_yaxis()
    ax.grid(axis="x", alpha=.2)
    ax.legend(fontsize=8)
axes[0].set(title="Wind estimate error: last 20 s before final", xlabel="Horizontal vector RMSE (m/s)")
axes[1].set(title="Contact location: negative = short", xlabel="Along-track error (m)")
fig.suptitle("One-line sideslip threshold test — unchanged landing settings", fontsize=14)
fig.savefig(HERE / "threshold-comparison.png", dpi=160)
for r in rows:
    print(r["id"], "truth", [round(v, 3) for v in r["truth_mean_ned_m_s"]],
          "estimate", [round(v, 3) for v in r["estimate_mean_ned_m_s"]],
          "wind RMSE", round(r["horizontal_wind_rmse_m_s"], 3),
          "GS RMSE", round(r["ground_speed_rmse_m_s"], 3),
          "miss NE", round(r["along_contact_error_m"], 2), round(r["cross_contact_error_m"], 2))

