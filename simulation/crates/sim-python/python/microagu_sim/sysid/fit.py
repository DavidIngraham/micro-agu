"""Bounded batch output-error identification, with training-only nuisance states."""

import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .data import validate_split
from .metrics import legacy_residual, response_residual
from .model import BOUNDS, DEFAULT, replay


@dataclass(frozen=True)
class FitOptions:
    objective: str = "response"
    fit_nuisance: bool = False
    max_replays: int = 2000
    workers: int = 4
    substeps: int = 32
    regularization: float = 0.005
    nuisance_weight: float = 0.02
    fd_step: float = 0.001

    def __post_init__(self):
        if self.objective not in ("legacy", "response"):
            raise ValueError("Unknown objective")
        if self.max_replays < 1 or self.workers < 1 or self.substeps < 1:
            raise ValueError("Positive budgets required")
        if not all(
            np.isfinite(v) and v > 0 for v in [self.fd_step, self.nuisance_weight]
        ):
            raise ValueError("Positive finite difference and nuisance scales required")
        if not np.isfinite(self.regularization) or self.regularization < 0:
            raise ValueError("Invalid regularization")


def _job(args):
    segment, theta, nuisance, options = args
    predicted, _, _ = replay(segment, theta, nuisance, substeps=options.substeps)
    obs = np.rad2deg(segment.observation[:, 3:6])
    fn = legacy_residual if options.objective == "legacy" else response_residual
    r = fn(obs, predicted)
    if options.fit_nuisance:
        r = np.r_[
            r,
            np.array(nuisance)
            / np.array([0.1, segment.wind_sigma, segment.wind_sigma])
            * np.sqrt(options.nuisance_weight / 3),
        ]
    return r


class BudgetExceeded(Exception):
    pass


class Objective:
    """Bounded cache shares unaffected windows across nuisance derivative probes."""

    def __init__(self, segments, options, pool):
        self.segments = segments
        self.options = options
        self.pool = pool
        self.replays = 0
        self.cache = {}
        self.trace = []
        self.best = None
        self.low = BOUNDS[:, 0].copy()
        self.high = BOUNDS[:, 1].copy()
        if options.fit_nuisance:
            self.low = np.r_[self.low, np.tile([-0.1, -2.0, -2.0], len(segments))]
            self.high = np.r_[self.high, np.tile([0.1, 2.0, 2.0], len(segments))]

    def physical(self, z):
        return self.low + np.asarray(z) * (self.high - self.low)

    def __call__(self, z):
        values = self.physical(z)
        theta = values[:6]
        nuisance = (
            values[6:].reshape(-1, 3)
            if self.options.fit_nuisance
            else np.zeros((len(self.segments), 3))
        )
        keys = [(i, *theta, *n) for i, n in enumerate(nuisance)]
        missing = [i for i, k in enumerate(keys) if k not in self.cache]
        if self.replays + len(missing) > self.options.max_replays:
            raise BudgetExceeded()
        jobs = [(self.segments[i], theta, nuisance[i], self.options) for i in missing]
        results = list(self.pool.map(_job, jobs))
        self.replays += len(jobs)
        for i, r in zip(missing, results):
            self.cache[keys[i]] = r
        r = np.r_[
            np.concatenate([self.cache[k] for k in keys]) / np.sqrt(len(keys)),
            np.log(theta[:5]) * np.sqrt(self.options.regularization / 5),
        ]
        cost = float(r @ r)
        row = {
            "replays": self.replays,
            "cost": cost,
            "theta": theta.tolist(),
            "nuisance": {s.name: n.tolist() for s, n in zip(self.segments, nuisance)},
        }
        self.trace.append(row)
        if self.best is None or cost < self.best["cost"]:
            self.best = row
        if len(self.cache) > 250:
            keep = {k: self.cache[k] for k in keys}
            self.cache = keep
        return r

    def jac(self, z):
        cols = []
        for j in range(len(z)):
            lo, hi = np.array(z).copy(), np.array(z).copy()
            lo[j] = max(0, z[j] - self.options.fd_step)
            hi[j] = min(1, z[j] + self.options.fd_step)
            cols.append((self(hi) - self(lo)) / (hi[j] - lo[j]))
        return np.column_stack(cols)


def fit(segments, options=None, starts=None):
    options = FitOptions() if options is None else options
    validate_split(segments)
    training = [s for s in segments if s.role == "train"]
    starts = [DEFAULT, [1, 1, 0.5, 0.5, 1, 0.04]] if starts is None else starts
    if not starts:
        raise ValueError("At least one starting point required")
    runs = []
    with ProcessPoolExecutor(max_workers=options.workers) as pool:
        for start in starts:
            obj = Objective(training, options, pool)
            x = (
                np.r_[np.asarray(start, float), np.zeros(3 * len(training))]
                if options.fit_nuisance
                else np.asarray(start, float)
            )
            if (
                x.shape != obj.low.shape
                or not np.isfinite(x).all()
                or np.any(x < obj.low)
                or np.any(x > obj.high)
            ):
                raise ValueError("Invalid starting parameters")
            z = (x - obj.low) / (obj.high - obj.low)
            started = time.monotonic()
            try:
                result = least_squares(
                    obj,
                    z,
                    jac=obj.jac,
                    bounds=(np.zeros(len(z)), np.ones(len(z))),
                    ftol=0.001,
                    xtol=0.001,
                    gtol=1e-6,
                    max_nfev=options.max_replays,
                )
                status = {
                    "success": bool(result.success),
                    "message": str(result.message),
                }
            except BudgetExceeded:
                status = {"success": False, "message": "Replay budget exhausted"}
            if obj.best is None:
                raise ValueError("Budget cannot evaluate all training windows")
            runs.append(
                {
                    "best": obj.best,
                    "status": status,
                    "replays": obj.replays,
                    "seconds": time.monotonic() - started,
                    "trace": obj.trace,
                }
            )
    return {
        "schema_version": 1,
        "training_names": [s.name for s in training],
        "training_episodes": sorted({s.episode for s in training}),
        "selection": min((r["best"] for r in runs), key=lambda r: r["cost"]),
        "runs": runs,
        "production_changed": False,
    }
