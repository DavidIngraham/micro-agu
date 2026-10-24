"""Measure landing accuracy and transitions; simulator truth is evaluation only."""
import hashlib
import json
import math
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


rows = []
traces = {}
for spec in json.loads((HERE / "cases.json").read_text(encoding="utf-8-sig")):
    path = HERE / "runs" / (spec["id"] + "-result.json")
    if not path.exists():
        raise FileNotFoundError(f"Missing current case result: {path}")
    data = json.loads(path.read_text())
    if data["spec"] != spec:
        raise ValueError(f"Result belongs to different case inputs: {path}")
    if data["failure"] or not data["contact"]:
        raise RuntimeError((spec["id"], data["failure"]))
    incoming = data["contact"]["incoming"]
    state = incoming["state"]
    airborne = [s for s in data["samples"] if s["time"] <= incoming["time"]]
    transitions = {}
    for phase in ("approach", "final", "flare"):
        sample = next(s for s in airborne if s["telemetry"].get("landing", {}).get("phase") == phase)
        transitions[phase] = {
            "time_s": sample["time"],
            "truth_height_m": -sample["state"][2],
            "north_m": sample["state"][0],
            "east_m": sample["state"][1],
            "sink_m_s": sample["state"][9],
            "hagl_m": sample["lidar"]["hagl"],
            "time_to_contact_s": incoming["time"] - sample["time"],
        }
    n, e = state[0] - data["target"]["north"], state[1] - data["target"]["east"]
    heading = data["target"]["heading"] if not spec.get("fixedMission") else 0
    row = {
        "id": spec["id"], "spec": spec,
        "spot_error_m": math.hypot(n, e),
        "along_error_m": n * math.cos(heading) + e * math.sin(heading),
        "cross_error_m": -n * math.sin(heading) + e * math.cos(heading),
        "sink_m_s": state[9], "ground_speed_m_s": math.hypot(state[7], state[8]),
        "contact_height_m": -state[2], "transitions": transitions,
        "modes": sorted(set(s["mode"] for s in airborne)),
        "final_phase": data["finalLanding"]["phase"],
        "flare_logs": [s for s in data["logs"] if s["text"].startswith("Flare ")],
        "result_sha256": digest(path),
    }
    assert row["modes"] == [10] and row["final_phase"] == "landed", row
    rows.append(row)
    traces[spec["id"]] = (airborne, incoming["time"])
(HERE / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")

files = [
    "viewer/generated/ardupilot/arduplane.js", "viewer/generated/ardupilot/arduplane.wasm",
    "viewer/generated/physics/sim_wasm.js", "viewer/generated/physics/sim_wasm_bg.wasm",
    "aircraft/microagu/configurations/browser-physics.json", "aircraft/microagu/controllers/lqi-gains.bin",
    "viewer/assets/runtime/build.json", "viewer/packages/sim-browser/src/session/runtime.ts",
    "viewer/packages/sim-browser/src/physics/physics.ts", "viewer/packages/sim-browser/src/physics/lidar.ts",
    "viewer/apps/simulator/missions/default.ts", "viewer/apps/simulator/weather.ts",
    "research/active/native-landing-2026-10-20/case.mts",
]
manifest = json.loads((ROOT / "viewer/assets/runtime/build.json").read_text())
live = json.loads((HERE / "live-build.json").read_text())
runtime_inputs = {
    "arduplane.wasm": "viewer/generated/ardupilot/arduplane.wasm",
    "arduplane.js": "viewer/generated/ardupilot/arduplane.js",
    "sim_wasm_bg.wasm": "viewer/generated/physics/sim_wasm_bg.wasm",
    "sim_wasm.js": "viewer/generated/physics/sim_wasm.js",
    "highdof.json": "aircraft/microagu/configurations/browser-physics.json",
    "lqi-gains.bin": "aircraft/microagu/controllers/lqi-gains.bin",
}
runtime_matches = {name: digest(ROOT / path) == live["sha256"][name]
                   for name, path in runtime_inputs.items()}
assert manifest == live and all(runtime_matches.values()), runtime_matches
provenance = {
    "microagu_revision": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
    "working_tree_status": subprocess.check_output(["git", "status", "--short"], cwd=ROOT, text=True),
    "firmware_revision": manifest["revision"], "live_manifest_identical": manifest == live,
    "runtime_inputs_match_live": runtime_matches,
    "live_manifest_url": "https://davidingraham.github.io/assets/microagu-viewer/wasm/build.json",
    "files": {f: digest(ROOT / f) for f in files},
    "study_files": {f: digest(HERE / f) for f in ("case.mts", "case-v1.mts", "run.py", "cases.json", "assess.py", "probe.mts", "parameters.json", "RESULTS.md")},
    "driver_versions": {
        "case-v1.mts": "First six cases; cases.json order. Same driver with parameter/weather overrides.",
        "case.mts": "Last four cases; adds passive PARAM_VALUE recording. No parameter request or control change.",
    },
    "evaluation": "Last sample before payload CG <= 0.12 m or canopy CG <= 0.05 m; not impact or rollout.",
}
(HERE / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
series = [
    ("mission-calm", "Current", "#38495b"),
    ("mission-lower-brake", "Brake at 0.8 m", "#c86b1d"),
    ("mission-lower-final", "Power-off at 1.5 m", "#9853a1"),
    ("mission-no-aim-offset", "Aim adjustment 0%", "#008366"),
]
for key, label, color in series:
    samples, end = traces[key]
    close = [s for s in samples if end - 5 <= s["time"] <= end]
    axes[0, 0].plot([s["state"][0] for s in close], [-s["state"][2] for s in close], label=label, color=color)
    for phase, marker in [("final", "o"), ("flare", "^")]:
        row = next(r for r in rows if r["id"] == key)["transitions"][phase]
        axes[0, 0].scatter(row["north_m"], row["truth_height_m"], marker=marker, color=color, s=40)
    axes[0, 1].plot([s["time"] - end for s in close], [s["state"][9] for s in close], color=color, label=label)
axes[0, 0].axvline(0, color="black", ls=":", lw=1)
axes[0, 0].set(xlabel="Distance along final (m; target = 0)", ylabel="Payload CG height (m)", ylim=(0, 5), title="Geographic timing: circle = final, triangle = brake flare")
axes[0, 0].legend(fontsize=8)
axes[0, 1].set(xlabel="Seconds before contact", ylabel="Downward speed (m/s)", xlim=(-3, 0), title="Delaying brake application makes contact faster")
samples, end = traces["mission-calm"]
close = [s for s in samples if end - 4 <= s["time"] <= end]
axes[1, 0].plot([s["time"] - end for s in close], [s["command"][0] * 100 for s in close], label="Throttle (%)", color="#38495b")
axes[1, 0].plot([s["time"] - end for s in close], [sum(s["brakes"]) / 2 for s in close], label="Applied mean brake (mm)", color="#008366")
axes[1, 0].set(xlabel="Seconds before contact", ylabel="Percent / millimetres", title="Current: power-off precedes the brake pull")
axes[1, 0].legend(fontsize=8)
keys = ["mission-calm", "mission-default-weather", "mission-lower-brake", "mission-lower-final", "mission-no-aim-offset", "mission-sink-plan"]
labels = ["Current calm", "Current weather", "Brake 0.8 m", "Final 1.5 m", "Aim 0%", "Plan sink 1.4"]
values = [next(r for r in rows if r["id"] == key)["along_error_m"] for key in keys]
axes[1, 1].barh(labels, values, color=["#38495b", "#788a9b", "#c86b1d", "#9853a1", "#008366", "#5475b5"])
axes[1, 1].axvline(0, color="black", lw=1)
axes[1, 1].invert_yaxis()
axes[1, 1].set(xlabel="Along-track contact error (m; negative = short)", title="Single-case sensitivities; no production tune changed")
for ax in axes.flat:
    ax.grid(alpha=.18)
fig.suptitle("MicroAGU spot landing — packaged web runtime, 23 Oct 2026", fontsize=15)
fig.savefig(HERE / "comparison.png", dpi=160)
print(json.dumps([{k: r[k] for k in ("id", "along_error_m", "cross_error_m", "sink_m_s", "ground_speed_m_s")} for r in rows], indent=2))
