"""Conditional finite-difference identifiability; not statistical confidence."""

from concurrent.futures import ProcessPoolExecutor

import numpy as np

from .fit import FitOptions, _job
from .model import BOUNDS


def sensitivity(segments, theta, nuisance=None, workers=4):
    training = [s for s in segments if s.role == "train"]
    theta = np.asarray(theta, float)
    width = BOUNDS[:, 1] - BOUNDS[:, 0]
    z = (theta - BOUNDS[:, 0]) / width
    nuisance = {} if nuisance is None else nuisance
    options = FitOptions(objective="response", regularization=0)
    with ProcessPoolExecutor(max_workers=workers) as pool:

        def evaluate(x):
            t = BOUNDS[:, 0] + x * width
            jobs = [
                (s, t, nuisance.get(s.name, [0.0, 0.0, 0.0]), options) for s in training
            ]
            return np.concatenate(list(pool.map(_job, jobs))) / np.sqrt(len(training))

        columns = []
        for k in range(6):
            low, high = z.copy(), z.copy()
            low[k] = max(0, z[k] - 0.001)
            high[k] = min(1, z[k] + 0.001)
            columns.append((evaluate(high) - evaluate(low)) / (high[k] - low[k]))
    j = np.column_stack(columns)
    norms = np.linalg.norm(j, axis=0)
    _, sv, vt = np.linalg.svd(j, full_matrices=False)
    return {
        "column_norms": norms.tolist(),
        "singular_values": sv.tolist(),
        "inactive_columns": np.flatnonzero(norms < 1e-8).tolist(),
        "condition_number": float(sv[0] / sv[-1]) if sv[-1] > 1e-12 else None,
        "weakest_direction": vt[-1].tolist(),
        "bound_hits": np.flatnonzero((z < 0.01) | (z > 0.99)).tolist(),
        "interpretation": "Local conditional response sensitivity, normalized by exploratory bounds. Nuisance fixed; no posterior confidence or global-identifiability claim.",
    }
