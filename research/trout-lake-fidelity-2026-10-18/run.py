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

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "historical", ROOT / "research/rigging-refinement-2026-10-15/study.py"
)
historical = importlib.util.module_from_spec(spec)
spec.loader.exec_module(historical)
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
    # Flight-specific existing mass/inertia/CG retained. TL zero correction does not transfer to HR.
    if flight == "trout_lake":
        sim.parameters["left_offset"] += theta[0] / 1000
        sim.parameters["right_offset"] += theta[1] / 1000
    sim.parameters["brake_cp_shift"] = theta[2]
    sim.parameters["Cndelta"] = theta[3]
    for key, value in zip(
        ["joint_k_roll", "joint_c_pitch", "joint_c_yaw", "CLbrake", "CDbrake"],
        theta[4:],
    ):
        sim.parameters[key] *= value
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


TRAIN_BLOCKS = [660, 1140, 1740, 2520]
CHECK_BLOCKS = [1440, 2100, 2760]
TRAIN = [("trout_lake", t + d) for t in TRAIN_BLOCKS for d in [0, 30, 60]]
CHECK = [("trout_lake", t + d) for t in CHECK_BLOCKS for d in [0, 30, 60]]
TRANSFER = [("hood_river", t) for t in [360, 600, 720, 1020, 438]]
NAMES = [
    "left_zero_mm",
    "right_zero_mm",
    "brake_cp_shift",
    "Cndelta",
    "roll_stiffness_scale",
    "pitch_damping_scale",
    "yaw_damping_scale",
    "CLbrake_scale",
    "CDbrake_scale",
]
BOUNDS = [
    (-3, 3),
    (-3, 3),
    (0, 0.3),
    (0, 0.035),
    (0.5, 1.5),
    (0.5, 2),
    (0.5, 2),
    (0.7, 1.3),
    (0.7, 1.5),
]
DEFAULT = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2) + "\n")


def main():
    save(
        "protocol.json",
        dict(
            primary="trout_lake",
            geometry_mm=132,
            mass_kg=1.758,
            train_blocks=TRAIN_BLOCKS,
            check_blocks=CHECK_BLOCKS,
            block_duration_s=90,
            window_duration_s=30,
            training=TRAIN,
            checks=CHECK,
            transfer=TRANSFER,
            names=NAMES,
            bounds=BOUNDS,
            filter_hz=0.7,
            metrics=METRICS,
            scales=SCALES.tolist(),
            group_weights=[0.4, 0.4, 0.2],
            initial_alpha_fit=0.35,
            initial_alpha_checks=[0.25, 0.35, 0.45],
            selection="Lowest regularized training objective across both families, frozen before check replay",
            optimizer="Bounded Powell, two starts per family, 120 evaluations/start zero-only, 240/start physics",
            gates=dict(
                mean_ratio=0.95,
                worst_window_ratio=1.15,
                max_group_ratio=1.05,
                min_improved_windows=6,
                each_alpha_mean_ratio=0.95,
            ),
            limitations="Contiguous 90 s blocks split into independently initialized 30 s replays. Same historical flight, potentially previously examined in other studies; not independent flights. Hood River is diagnostic only. No automatic production deployment.",
        ),
    )
    save(
        "input-hashes.json",
        {
            p.name: hashlib.sha256(p.read_bytes()).hexdigest()
            for day, _ in historical.FLIGHTS.values()
            for p in [
                historical.DATA / f"{day}-causal-input.npz",
                historical.DATA / f"{day}-propulsion-9x45.npz",
            ]
        },
    )
    # Verify split and validity before any simulation or fitting.
    assert all(abs(t - v) >= 120 for _, t in TRAIN for _, v in CHECK)
    for f, t in TRAIN + CHECK + TRANSFER:
        load(f, t)
    trace = []
    selections = {}
    optimizers = []
    with ProcessPoolExecutor(max_workers=4) as pool:

        def batch(cases, theta, alpha=0.35, keep=""):
            return list(
                pool.map(evaluate, [(f, t, theta, alpha, keep) for f, t in cases])
            )

        baseline = batch(TRAIN, DEFAULT)
        save("baseline-training.json", baseline)
        for family in ["zero-only", "physics"]:
            n = 2 if family == "zero-only" else 9
            for seed in [0, 1]:
                x = DEFAULT[:n].copy()
                if seed:
                    x[:2] = [1.0, -1.0]
                    if n == 9:
                        x[2:] = [0.06, 0.008, 0.8, 1.3, 1.3, 1.0, 1.2]

                def objective(x):
                    theta = list(x) + DEFAULT[n:]
                    rows = batch(TRAIN, theta)
                    ratio = float(
                        np.mean(
                            [r["score"] / b["score"] for r, b in zip(rows, baseline)]
                        )
                    )
                    penalty = 0.005 * float(np.mean((np.array(theta[:2]) / 3) ** 2))
                    if n == 9:
                        penalty += 0.005 * float(
                            np.mean(np.log(np.array(theta[4:])) ** 2)
                        )
                    row = dict(
                        family=family,
                        seed=seed,
                        theta=theta,
                        ratio=ratio,
                        objective=ratio + penalty,
                        finite=all(r["finite"] for r in rows),
                    )
                    trace.append(row)
                    if len(trace) % 20 == 0:
                        print(
                            "evaluations", len(trace), family, seed, ratio, flush=True
                        )
                    return ratio + penalty

                result = minimize(
                    objective,
                    x,
                    method="Powell",
                    bounds=BOUNDS[:n],
                    options=dict(maxfev=120 if n == 2 else 240, xtol=0.01, ftol=0.001),
                )
                optimizers.append(
                    dict(
                        family=family,
                        seed=seed,
                        success=bool(result.success),
                        message=str(result.message),
                        nfev=int(result.nfev),
                    )
                )
                save("optimization.json", trace)
                save("optimizer-status.json", optimizers)
            selections[family] = min(
                [r for r in trace if r["family"] == family and r["finite"]],
                key=lambda r: r["objective"],
            )
            print("Selected", family, selections[family], flush=True)
        selected = min(selections.values(), key=lambda r: r["objective"])
        save("selection.json", dict(families=selections, primary=selected))
        checks = []
        for family, theta in [("baseline", DEFAULT)] + [
            (k, v["theta"]) for k, v in selections.items()
        ]:
            for alpha in [0.25, 0.35, 0.45]:
                checks.extend(
                    dict(r, family=family)
                    for r in batch(CHECK, theta, alpha, keep=family)
                )
        save("checks.json", checks)
        transfer = []
        for family, theta in [("baseline", DEFAULT), ("selected", selected["theta"])]:
            transfer.extend(
                dict(r, family=family)
                for r in batch(TRANSFER, theta, keep="transfer-" + family)
            )
        save("transfer.json", transfer)
    report(checks, selected, transfer)


def report(rows, selected, transfer):
    base = {(r["start"], r["alpha"]): r for r in rows if r["family"] == "baseline"}
    summary = {}
    for family in ["zero-only", "physics"]:
        rs = [r for r in rows if r["family"] == family]
        ratios = [
            float(
                np.mean(
                    [
                        r["score"] / base[t, r["alpha"]]["score"]
                        for r in rs
                        if r["start"] == t
                    ]
                )
            )
            for _, t in CHECK
        ]
        groups = [
            float(
                np.mean(
                    [
                        r["groups"][j] / base[r["start"], r["alpha"]]["groups"][j]
                        for r in rs
                    ]
                )
            )
            for j in range(3)
        ]
        alphas = [
            float(
                np.mean(
                    [
                        r["score"] / base[r["start"], a]["score"]
                        for r in rs
                        if r["alpha"] == a
                    ]
                )
            )
            for a in [0.25, 0.35, 0.45]
        ]
        passed = bool(
            np.mean(ratios) <= 0.95
            and max(ratios) <= 1.15
            and max(groups) <= 1.05
            and sum(x < 1 for x in ratios) >= 6
            and max(alphas) <= 0.95
            and all(r["finite"] for r in rs)
        )
        summary[family] = dict(
            mean_ratio=float(np.mean(ratios)),
            window_ratios=ratios,
            group_ratios=groups,
            alpha_ratios=alphas,
            improved_windows=sum(x < 1 for x in ratios),
            passed=passed,
        )
    summary["selected_family"] = selected["family"]
    summary["selected_passed"] = summary[selected["family"]]["passed"]
    summary["production_changed"] = False
    base = {r["start"]: r for r in transfer if r["family"] == "baseline"}
    summary["hood_river_transfer_ratios"] = {
        r["start"]: r["score"] / base[r["start"]]["score"]
        for r in transfer
        if r["family"] == "selected"
    }
    save("summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
