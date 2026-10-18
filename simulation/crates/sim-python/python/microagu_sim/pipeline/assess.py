"""Paired nonlinear metrics: stability gates precede tracking and shaft-power ranking."""

import json
import re
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def power_table(path):
    text = Path(path).read_text()

    def table(name):
        match = re.search(rf"{name}: \[f64; \d+\] = \[(.*?)\];", text, re.DOTALL)
        if not match:
            raise ValueError("Missing propeller table " + name)
        return np.array([float(x) for x in match[1].split(",") if x.strip()])

    return table("RPM_GRID"), table("CP_GRID").reshape(25, 401)


def shaft_power(rpm, axial, grid, cp_table):
    if np.any(rpm < grid[0]) or np.any(rpm >= grid[-1]):
        raise ValueError("RPM outside characterized power map")
    i = np.clip(np.searchsorted(grid, rpm, side="right") - 1, 0, 23)
    f = (rpm - grid[i]) / (grid[i + 1] - grid[i])
    advance = (
        np.clip(np.maximum(axial, 0) / (np.maximum(rpm, 1e-9) / 60 * 0.2286), 0, 2)
        * 200
    )
    k = np.minimum(advance.astype(int), 399)
    g = advance - k
    cp = (cp_table[i, k] * (1 - g) + cp_table[i, k + 1] * g) * (1 - f) + (
        cp_table[i + 1, k] * (1 - g) + cp_table[i + 1, k + 1] * g
    ) * f
    return 1.225 * (rpm / 60) ** 3 * 0.2286**5 * cp


def assess(path, table):
    data = json.loads(Path(path).read_text())
    ss = data["samples"]
    gates = []
    if not data["complete"] or data["failure"] or len(ss) < 100:
        return {"gates": [data["failure"] or "Incomplete run"], "metrics": None}
    t = np.array([s["time"] for s in ss])
    y = np.array([s["truth"] for s in ss])
    if not np.isfinite(y).all() or t[-1] < 319 or np.any(np.diff(t) <= 0):
        return {"gates": ["Invalid or incomplete trajectory"], "metrics": None}
    rp = Rotation.from_quat(y[:, 3:7])
    rc = Rotation.from_quat(y[:, 16:20])
    attitude = rp.as_euler("xyz", degrees=True)
    relative = (rp.inv() * rc).as_rotvec() * 180 / np.pi
    rates = rp.inv().apply(y[:, 23:26] - y[:, 10:13]) * 180 / np.pi
    altitude = -y[:, 2]
    target = np.array([s["requestedAltitude"] for s in ss])
    error = altitude - target
    brakes = np.array([s["brakes"] for s in ss]) / (
        np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1
    )
    saturation = float(np.mean(np.any(brakes > 0.99, axis=1)))
    if (
        max(abs(attitude[:, 0])) > 60
        or max(abs(attitude[:, 1])) > 45
        or min(altitude) < 3
    ):
        gates.append("Attitude/altitude envelope")
    if any(not s["active"] for s in ss if s["time"] > 5):
        gates.append("LQI ownership lost")
    if saturation > 0.05:
        gates.append("Upper brake saturation >5%")
    rms = lambda x: float(np.sqrt(np.mean(x * x)))
    late = t > 300
    late_angle = rms(relative[late, 1] - np.mean(relative[late, 1]))
    late_rate = rms(rates[late, 1])
    if late_angle > 1 and late_rate > 2:
        gates.append("Persistent longitudinal oscillation")
    tasks = {}
    for name, start, end in [
        ("climb", 20, 120),
        ("descent", 120, 220),
        ("vertical", 220, 260),
        ("crosswind", 260, 320),
    ]:
        mask = (t >= start) & (t < end)
        tail = mask & (t > end - 10)
        if rms(error[tail]) > 1:
            gates.append(name + " did not recover within 1 m")
        tasks[name] = rms(error[mask])
    wind = np.array([s["wind"] for s in ss])
    axial = rp.inv().apply(y[:, 7:10] - wind)[:, 0]
    watts = shaft_power(np.array([s["rpm"] for s in ss]), axial, *table)
    distance = np.trapezoid(np.linalg.norm(y[:, 7:9], axis=1), t)
    metrics = dict(
        **tasks,
        relative_pitch_rate_rms=rms(rates[:, 1]),
        relative_pitch_rate_peak=float(max(abs(rates[:, 1]))),
        late_relative_pitch_rms=late_angle,
        late_relative_pitch_rate_rms=late_rate,
        upper_brake_fraction=saturation,
        common_brake=float(np.mean(brakes.min(axis=1))),
        shaft_W=float(np.trapezoid(watts, t) / (t[-1] - t[0])),
        shaft_Wh_per_km=float(np.trapezoid(watts, t) / 3.6 / distance),
    )
    return {"gates": gates, "metrics": metrics}


def compare(rows, gates=None):
    gates = gates or {
        "tracking_improvement": 0.05,
        "task_max_regression": 0.2,
        "motion_max_regression": 0.1,
        "power_max_regression": 0.05,
    }
    names = sorted(
        {row["controller"] for row in rows} - {"reference", "reference_repeat"}
    )
    out = {}
    base = {r["case"]: r for r in rows if r["controller"] == "reference"}
    repeat = {r["case"]: r for r in rows if r["controller"] == "reference_repeat"}
    repeat_reasons = []
    if set(repeat) != set(base):
        repeat_reasons.append("Missing identical-gain reference repeat")
    for case in sorted(set(repeat) & set(base)):
        b, r = base[case], repeat[case]
        if b["gates"] or r["gates"] or b["metrics"] is None or r["metrics"] is None:
            repeat_reasons.append(case + ": reference repeat failed")
            continue
        for metric, tolerance, floor in [
            ("climb", 0.1, 0.25),
            ("descent", 0.1, 0.25),
            ("vertical", 0.1, 0.25),
            ("crosswind", 0.1, 0.25),
            ("relative_pitch_rate_rms", 0.1, 1),
            ("relative_pitch_rate_peak", 0.1, 10),
            ("shaft_W", 0.03, 5),
            ("shaft_Wh_per_km", 0.03, 0.2),
        ]:
            delta = abs(r["metrics"][metric] - b["metrics"][metric])
            if delta > max(floor, tolerance * abs(b["metrics"][metric])):
                repeat_reasons.append(case + ": nonrepeatable " + metric)
    for name in names:
        group = [r for r in rows if r["controller"] == name]
        reasons = list(repeat_reasons)
        ratios = []
        if {r["case"] for r in group} != set(base):
            reasons.append("Unpaired cases")
        for r in group:
            b = base[r["case"]]
            reasons.extend(f"{r['case']}: {g}" for g in r["gates"])
            reasons.extend(f"reference {r['case']}: {g}" for g in b["gates"])
            if r["metrics"] is None or b["metrics"] is None:
                continue
            ratio = {
                k: r["metrics"][k] / max(b["metrics"][k], 0.01)
                for k in [
                    "climb",
                    "descent",
                    "vertical",
                    "crosswind",
                    "relative_pitch_rate_rms",
                    "relative_pitch_rate_peak",
                    "shaft_W",
                    "shaft_Wh_per_km",
                ]
            }
            ratios.append(ratio)
            if any(
                ratio[k] > 1 + gates["task_max_regression"]
                for k in ["climb", "descent", "vertical", "crosswind"]
            ):
                reasons.append(r["case"] + ": task regression >20%")
            if (
                max(ratio["relative_pitch_rate_rms"], ratio["relative_pitch_rate_peak"])
                > 1 + gates["motion_max_regression"]
            ):
                reasons.append(r["case"] + ": motion regression >10%")
            if (
                max(ratio["shaft_W"], ratio["shaft_Wh_per_km"])
                > 1 + gates["power_max_regression"]
            ):
                reasons.append(r["case"] + ": power/energy regression >5%")
        score = (
            float(
                np.mean(
                    [
                        np.mean(
                            [
                                v[k]
                                for k in ["climb", "descent", "vertical", "crosswind"]
                            ]
                        )
                        for v in ratios
                    ]
                )
            )
            if ratios
            else None
        )
        if score is None or score > 1 - gates["tracking_improvement"]:
            reasons.append("Tracking improvement below 5%")
        out[name] = {
            "reasons": reasons,
            "paired_ratios": ratios,
            "mean_tracking_ratio": score,
            "eligible": not reasons,
        }
    eligible = [n for n in names if out[n]["eligible"]]
    return {
        "repeatability": {"passed": not repeat_reasons, "reasons": repeat_reasons},
        "selected": min(eligible, key=lambda n: out[n]["mean_tracking_ratio"])
        if eligible
        else None,
        "candidates": out,
        "deploy": False,
        "note": "Training selection only; independent reserved native scenarios and mission checks still required.",
    }
