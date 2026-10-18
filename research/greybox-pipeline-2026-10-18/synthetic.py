"""Known-physics recovery and controlled wind-error experiment on Rust dynamics."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.sysid.data import load_session
from microagu_sim.sysid.diagnostics import sensitivity
from microagu_sim.sysid.fit import FitOptions, fit
from microagu_sim.sysid.metrics import response_residual
from microagu_sim.sysid.model import BOUNDS, replay
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
TRUTH = np.array([1.2, 0.95, 0.65, 0.7, 1.04, 0.035])


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def generate(segments, wind_error, noise):
    rng = np.random.default_rng(20261018)
    result = []
    for s in segments:
        if s.name not in ("tl-280", "tl-535", "tl-440"):
            continue
        source = replace(s, wind=np.array([1.0, -0.5, 0.0]))
        pred, y, _ = replay(source, TRUTH)
        rp = Rotation.from_quat(y[:, 3:7])
        imu = make_simulator(s.flight, profile="as_built").geometry["imu"]
        obs = np.column_stack(
            [
                rp.as_euler("xyz"),
                np.deg2rad(pred),
                y[:, 7:10] + np.cross(y[:, 10:13], rp.apply(imu)),
            ]
        )
        obs[:, 3:6] += rng.normal(0, np.deg2rad(noise), obs[:, 3:6].shape)
        result.append(
            replace(
                source,
                observation=obs,
                initial_state=y[0].copy(),
                wind=source.wind + np.r_[wind_error, 0.0],
            )
        )
    return result


def main():
    _, source, _ = load_session(OUT / "session.json")
    save(
        "synthetic-protocol.json",
        {
            "truth": TRUTH.tolist(),
            "training": ["tl-280", "tl-535"],
            "validation": ["tl-440"],
            "cases": ["exact", "wind_bias_no_nuisance", "wind_bias_with_nuisance"],
            "noise_deg_s": 0.1,
            "wind_bias_m_s": [0.6, -0.4],
            "note": "Same simulator generates and fits data. Tests implementation and conditional recovery, not real-world model adequacy. Initial full state is known in synthetic cases; no fitted validation nuisance.",
        },
    )
    results = {}
    for name, error, noise, nuisance in [
        ("exact", [0, 0], 0, False),
        ("wind_bias_no_nuisance", [0.6, -0.4], 0.1, False),
        ("wind_bias_with_nuisance", [0.6, -0.4], 0.1, True),
    ]:
        data = generate(source, error, noise)
        options = FitOptions(
            objective="response",
            fit_nuisance=nuisance,
            max_replays=1800,
            regularization=0,
        )
        print("Synthetic", name, flush=True)
        result = fit(data, options, starts=[[1, 1, 0.8, 0.8, 1, 0.04]])
        selected = result["selection"]
        selected["truth"] = TRUTH.tolist()
        selected["normalized_parameter_error"] = float(
            np.linalg.norm(
                (np.array(selected["theta"]) - TRUTH) / (BOUNDS[:, 1] - BOUNDS[:, 0])
            )
        )
        selected["sensitivity"] = sensitivity(
            data, selected["theta"], selected["nuisance"]
        )
        # Model-only test on held-out synthetic trajectory with known environmental wind.
        s = next(s for s in data if s.role == "validation")
        s = replace(s, wind=np.array([1.0, -0.5, 0.0]))
        pred, _, _ = replay(s, selected["theta"])
        obs = np.rad2deg(s.observation[:, 3:6])
        r = response_residual(obs, pred)
        selected["known_wind_validation_response_loss"] = float(r @ r)
        results[name] = selected
        save(f"synthetic-fit-{name}.json", result)
        save("synthetic-summary.json", results)
        print(name, selected, flush=True)


if __name__ == "__main__":
    main()
