"""Frequency-separated, recorded-input identification of coupled brake loads.

Offline research only. Does not write presets, firmware gains or runtime assets.
"""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
from pathlib import Path
import hashlib
import importlib.util
import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import minimize
from scipy.signal import butter, sosfiltfilt
from scipy.spatial.transform import Rotation
from microagu_sim.compat.profiles import make_simulator
from microagu_sim import _native

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "historical", ROOT / "research/archive/rigging-refinement-2026-10-15/study.py"
)
historical = importlib.util.module_from_spec(spec)
spec.loader.exec_module(historical)
TRAIN = [
    (f, t) for f, (_, starts) in historical.FLIGHTS.items() for t in starts[::2]
] + [("hood_river", 438)]
CHECK = [(f, t) for f, (_, starts) in historical.FLIGHTS.items() for t in starts[1::2]]
NAMES = [
    "hr_left_zero_mm",
    "hr_right_zero_mm",
    "tl_left_zero_mm",
    "tl_right_zero_mm",
    "brake_cp_shift",
    "brake_outboard_fraction",
    "Cndelta",
]
BOUNDS = [(-3.0, 3.0)] * 4 + [(0.0, 0.3), (0.0, 1.0), (0.0, 0.035)]
METRICS = [
    "slow_bank",
    "slow_yaw_rate",
    "course",
    "slow_pitch",
    "slow_climb",
    "slow_speed",
    "fast_roll_rate",
    "fast_pitch_rate",
    "fast_yaw_rate",
]
SCALES = np.array([5.0, 10.0, 90.0, 5.0, 0.4, 1.0, 15.0, 15.0, 15.0])
FILTER = butter(3, 0.7, fs=25, output="sos")
DATA = {}


def load(flight, start):
    key = (flight, start)
    if key not in DATA:
        day = historical.FLIGHTS[flight][0]
        a = dict(np.load(historical.DATA / f"{day}-causal-input.npz"))
        b = dict(np.load(historical.DATA / f"{day}-propulsion-9x45.npz"))
        i = int(np.argmin(abs(a["t"] - start)))
        sl = slice(i, i + 751)
        assert len(a["t"][sl]) == 751 and a["valid"][sl].all()
        assert np.max(abs(a["t"] - b["t"])) < 1e-6
        assert np.max(abs(np.diff(a["t"][sl]) - 0.04)) < 1e-5
        DATA[key] = (a, b, i, sl)
    return DATA[key]


def channels(euler, rates, velocity):
    # The observations are at the IMU, so the simulation is transformed to that point.
    angles = euler[:, :2] * 180 / np.pi
    rate_deg = rates * 180 / np.pi
    course = np.rad2deg(np.unwrap(np.arctan2(velocity[:, 1], velocity[:, 0])))
    course -= course[0]
    slow = lambda x: sosfiltfilt(FILTER, x, axis=0)
    fast = rate_deg - slow(rate_deg)
    return np.column_stack(
        [
            slow(angles[:, 0]),
            slow(rate_deg[:, 2]),
            slow(course),
            slow(angles[:, 1]),
            slow(-velocity[:, 2]),
            slow(np.linalg.norm(velocity[:, :2], axis=1)),
            fast,
        ]
    )


def evaluate(job, substeps=32):
    flight, start, theta, alpha, keep = job
    a, b, i, sl = load(flight, start)
    obs = a["Y"][sl]
    sim = make_simulator(flight, profile="as_built", substeps=substeps)
    offset = 0 if flight == "hood_river" else 2
    sim.parameters["left_offset"] += theta[offset] / 1000
    sim.parameters["right_offset"] += theta[offset + 1] / 1000
    sim.parameters["brake_cp_shift"] = theta[4]
    sim.parameters["brake_outboard_fraction"] = theta[5]
    sim.parameters["Cndelta"] = theta[6]
    y0 = historical.init(sim, a, b, i, alpha)
    u = np.column_stack([a["U"][sl, 0], a["brake_mm"][sl]])
    row = {"flight": flight, "start": start, "alpha": alpha, "theta": list(theta)}
    try:
        y, rpm = sim.replay(
            y0,
            u,
            voltage=float(b["voltage"][i]),
            rpm=float(b["rpm"][i]),
            wind=a["wind"][i],
        )
        if not np.isfinite(y).all():
            raise FloatingPointError("Nonfinite trajectory")
        rp = Rotation.from_quat(y[:, 3:7])
        euler = rp.as_euler("xyz")
        rates = rp.inv().apply(y[:, 10:13])
        nav = y[:, 7:10] + np.cross(y[:, 10:13], rp.apply(sim.geometry["imu"]))
        actual = channels(obs[:, :3], obs[:, 3:6], obs[:, 6:9])
        predicted = channels(euler, rates, nav)
        mask = (
            np.arange(len(y)) >= 50
        )  # exclude first two seconds, fixed before optimization
        # Exclude two seconds at the end as well to avoid zero-phase filter edge effects.
        mask[-50:] = False
        rmse = np.sqrt(np.mean((predicted[mask] - actual[mask]) ** 2, axis=0))
        group = np.array(
            [
                np.mean((rmse[:3] / SCALES[:3]) ** 2),
                np.mean((rmse[3:6] / SCALES[3:6]) ** 2),
                np.mean((rmse[6:] / SCALES[6:]) ** 2),
            ]
        )
        row.update(
            finite=True,
            metrics=dict(zip(METRICS, rmse.tolist())),
            groups=group.tolist(),
            score=float(group @ np.array([0.4, 0.4, 0.2])),
        )
        if start == 438:
            turn = (a["t"][sl] >= 447) & (a["t"][sl] <= 456)

            def summary(roll, v):
                course = np.rad2deg(np.unwrap(np.arctan2(v[turn, 1], v[turn, 0])))
                return {
                    "bank_peak_deg": float(max(abs(roll[turn] * 180 / np.pi))),
                    "course_rotation_deg": float(course[-1] - course[0]),
                    "mean_climb_mps": float(-v[turn, 2].mean()),
                }

            row["turn"] = {
                "flight": summary(obs[:, 0], obs[:, 6:9]),
                "model": summary(euler[:, 0], nav),
            }
        if keep:
            np.savez_compressed(
                OUT / f"{keep}-{flight}-{start}-{alpha}.npz",
                time=a["t"][sl],
                observed=obs,
                model=y,
                model_velocity=nav,
                model_rates=rates,
                commands=u,
                filtered_observed=actual,
                filtered_model=predicted,
                rpm=rpm,
            )
    except (FloatingPointError, ValueError) as exc:
        row.update(finite=False, error=str(exc), score=1e6, groups=[1e6] * 3)
    return row


def main():
    protocol = {
        "geometry_mm": 132,
        "training": TRAIN,
        "checks": CHECK,
        "names": NAMES,
        "bounds": BOUNDS,
        "filter": {
            "sample_hz": 25,
            "lowpass_hz": 0.7,
            "order": 3,
            "zero_phase": True,
            "discard_edges_s": 2,
        },
        "metrics": METRICS,
        "scales": SCALES.tolist(),
        "group_weights": [0.4, 0.4, 0.2],
        "families": ["zero-only", "coupled"],
        "optimizer": "Powell; two specified starts per family; 96 function evaluations per start; no check-window access until selection frozen",
        "initial_alpha_fit": 0.35,
        "initial_alpha_checks": [0.25, 0.35, 0.45],
        "gates": {
            "mean_check_score_ratio": 0.95,
            "worst_window_score_ratio": 1.15,
            "improved_windows": 6,
            "maximum_mean_group_ratio": 1.1,
            "turn_course_relative_error": 0.2,
            "turn_peak_bank_relative_error": 0.15,
            "turn_mean_climb_absolute_error_mps": 0.15,
        },
        "limitations": "Reused historical checks, not independent flights. CP and spanwise loading are quasi-static hypotheses; offsets are bounded nuisance parameters, not inferred physical measurements. Production settings remain unchanged.",
    }
    (OUT / "protocol.json").write_text(json.dumps(protocol, indent=2) + "\n")
    hashes = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for day, _ in historical.FLIGHTS.values()
        for p in [
            historical.DATA / f"{day}-causal-input.npz",
            historical.DATA / f"{day}-propulsion-9x45.npz",
        ]
    }
    assert hashes == json.loads(
        (ROOT / "research/archive/as-built-simulator-2026-10-15/input-hashes.json").read_text()
    )
    (OUT / "input-hashes.json").write_text(json.dumps(hashes, indent=2) + "\n")
    trace = []
    selected = {}
    with ProcessPoolExecutor(max_workers=4) as pool:

        def batch(cases, theta, alpha=0.35, keep=""):
            return list(
                pool.map(evaluate, [(f, t, theta, alpha, keep) for f, t in cases])
            )

        baseline = batch(TRAIN, [0.0] * 7, keep="baseline")
        (OUT / "baseline-training.json").write_text(
            json.dumps(baseline, indent=2) + "\n"
        )
        for family in ["zero-only", "coupled"]:
            n = 4 if family == "zero-only" else 7
            for seed in [0, 1]:
                start = np.zeros(n)
                if seed == 1:
                    start[:4] = [1.0, -1.0, -1.0, 1.0]
                    if n == 7:
                        start[4:] = [0.08, 0.5, 0.015]

                def objective(x):
                    theta = np.r_[x, np.zeros(7 - n)].tolist()
                    rows = batch(TRAIN, theta)
                    fit = float(np.mean([r["score"] for r in rows]))
                    penalty = 0.02 * float(np.mean((np.array(theta[:4]) / 3) ** 2))
                    result = {
                        "family": family,
                        "seed": seed,
                        "theta": theta,
                        "fit": fit,
                        "penalty": penalty,
                        "objective": fit + penalty,
                        "all_finite": all(r["finite"] for r in rows),
                    }
                    trace.append(result)
                    if len(trace) % 20 == 0:
                        print(
                            f"evaluations {len(trace)}: {family} / {seed}, objective {fit + penalty:.5f}",
                            flush=True,
                        )
                    return fit + penalty

                minimize(
                    objective,
                    start,
                    method="Powell",
                    bounds=BOUNDS[:n],
                    options={"maxfev": 96, "xtol": 0.015, "ftol": 0.002},
                )
            candidates = [r for r in trace if r["family"] == family and r["all_finite"]]
            best = min(candidates, key=lambda r: r["objective"])
            selected[family] = best
            print("Selected", family, best, flush=True)
        # Freeze both selections before inspecting any check results.
        (OUT / "optimization.json").write_text(json.dumps(trace, indent=2) + "\n")
        (OUT / "selection.json").write_text(json.dumps(selected, indent=2) + "\n")
        rows = []
        for family, theta in [("baseline", [0.0] * 7)] + [
            (k, v["theta"]) for k, v in selected.items()
        ]:
            for alpha in [0.25, 0.35, 0.45]:
                rows.extend(
                    dict(row, family=family)
                    for row in batch(
                        CHECK + [("hood_river", 438)], theta, alpha, keep=family
                    )
                )
        (OUT / "checks.json").write_text(json.dumps(rows, indent=2) + "\n")
    report(rows, selected)


def report(rows, selected):
    base = {
        (r["flight"], r["start"], r["alpha"]): r
        for r in rows
        if r["family"] == "baseline"
    }
    summary = {}
    for family in ["zero-only", "coupled"]:
        group = [r for r in rows if r["family"] == family and r["start"] != 438]
        ratios = [
            float(
                np.mean(
                    [
                        r["score"] / base[f, t, r["alpha"]]["score"]
                        for r in group
                        if r["flight"] == f and r["start"] == t
                    ]
                )
            )
            for f, t in CHECK
        ]
        group_ratio = [
            float(
                np.mean(
                    [
                        r["groups"][j]
                        / base[r["flight"], r["start"], r["alpha"]]["groups"][j]
                        for r in group
                    ]
                )
            )
            for j in range(3)
        ]
        turns = [r for r in rows if r["family"] == family and r["start"] == 438]
        turn_ok = all(
            abs(
                r["turn"]["model"]["course_rotation_deg"]
                / r["turn"]["flight"]["course_rotation_deg"]
                - 1
            )
            <= 0.2
            and abs(
                r["turn"]["model"]["bank_peak_deg"]
                / r["turn"]["flight"]["bank_peak_deg"]
                - 1
            )
            <= 0.15
            and abs(
                r["turn"]["model"]["mean_climb_mps"]
                - r["turn"]["flight"]["mean_climb_mps"]
            )
            <= 0.15
            for r in turns
        )
        passed = bool(
            np.mean(ratios) <= 0.95
            and max(ratios) <= 1.15
            and sum(v < 1 for v in ratios) >= 6
            and max(group_ratio) <= 1.1
            and turn_ok
            and all(r["finite"] for r in group + turns)
        )
        summary[family] = {
            "mean_check_score_ratio": float(np.mean(ratios)),
            "window_ratios": ratios,
            "improved_windows": sum(v < 1 for v in ratios),
            "group_ratios_lateral_longitudinal_fast": group_ratio,
            "turn_gate": turn_ok,
            "passed": passed,
            "nominal_turn": next(r["turn"] for r in turns if r["alpha"] == 0.35),
        }
    summary["production_changed"] = False
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
