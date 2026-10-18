"""Numerical resolution and initialization-confounding checks; no fitting."""

import json
from concurrent.futures import ProcessPoolExecutor

import numpy as np
import run as study


def main():
    selected = json.loads((study.OUT / "selection.json").read_text())
    winners = {
        m: min(
            (k for k in selected if k.startswith(m)),
            key=lambda k: selected[k]["objective"],
        )
        for m in ["Powell", "least_squares"]
    }
    integration = []
    for tag in winners.values():
        for start in [380, 2760]:
            job = (start, selected[tag]["theta"], 0.35, "")
            a = study.prior.evaluate(job, substeps=32)
            b = study.prior.evaluate(job, substeps=64)
            integration.append(
                {
                    "family": tag,
                    "start": start,
                    "max_rate_rmse_difference_deg_s": float(
                        np.max(np.abs(np.array(a["rate_rmse"]) - b["rate_rmse"]))
                    ),
                    "phase_error_difference_deg": abs(
                        a["phase_error_deg"] - b["phase_error_deg"]
                    ),
                    "score_difference": abs(a["score"] - b["score"]),
                }
            )
    theta = json.loads((study.OLD / "constrained-selection.json").read_text())["theta"]
    with ProcessPoolExecutor(max_workers=4) as pool:
        lo = list(
            pool.map(study.worker, [(s, theta, 0.34, "") for s in study.prior.TRAIN])
        )
        hi = list(
            pool.map(study.worker, [(s, theta, 0.36, "") for s in study.prior.TRAIN])
        )
    j = np.load(study.OUT / "sensitivity-constrained.npz")["jacobian"]
    # Allow one unknown canopy initial angle per training window, independently.
    nuisance = np.zeros((len(j), len(lo)))
    cursor = 0
    for col, ((_, a), (_, b)) in enumerate(zip(lo, hi)):
        nuisance[cursor : cursor + len(a), col] = (b - a) / 0.02 / np.sqrt(len(lo))
        cursor += len(a)
    assert cursor == len(j)
    q, _ = np.linalg.qr(nuisance)
    projected = j - q @ (q.T @ j)
    old = np.linalg.svd(j, compute_uv=False)
    new = np.linalg.svd(projected, compute_uv=False)
    init = {
        "parameter_names": study.prior.NAMES,
        "conditional_singular_values": old.tolist(),
        "initial_angle_projected_singular_values": new.tolist(),
        "relative_column_norm_remaining": (
            np.linalg.norm(projected, axis=0) / np.linalg.norm(j, axis=0)
        ).tolist(),
        "condition_number_after_projection": float(new[0] / new[-1]),
        "interpretation": "Local residual Jacobian at previous constrained candidate. Project away independent canopy-initial-angle sensitivity per training window. No wind or other nuisance parameters included; not a confidence interval.",
    }
    study.save("audit.json", {"integration": integration, "initialization": init})
    print(json.dumps({"integration": integration, "initialization": init}, indent=2))


if __name__ == "__main__":
    main()
