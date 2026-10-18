"""Finite-difference identification study; no production model changes."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import hashlib
import importlib.util
import json
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, minimize
from scipy.signal import csd, savgol_filter, welch

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
OLD = ROOT / "research/active/manual-coupling-fit-2026-10-18"
spec = importlib.util.spec_from_file_location("coupling_prior", OLD / "run.py")
prior = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prior)
LOW, HIGH = np.array(prior.BOUNDS).T
WIDTH = HIGH - LOW
BUDGET = 220
STARTS = [prior.DEFAULT, [1.0, 1.0, 0.5, 0.5, 1.0, 0.04]]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def residual(obs, pred):
    a, m = obs[50:-10], pred[50:-10]
    e = m - a
    e -= e.mean(axis=0)
    da = savgol_filter(a, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    dm = savgol_filter(m, 11, 3, deriv=1, delta=0.04, axis=0)[6:-6]
    de = dm - da
    de -= de.mean(axis=0)
    rate = (e / np.maximum(a.std(axis=0), 1)).ravel() * np.sqrt(0.4 / (6 * len(a)))
    acc = (de / np.maximum(da.std(axis=0), 5)).ravel() * np.sqrt(0.4 / (6 * len(da)))
    f, po = welch(a, fs=25, nperseg=256, noverlap=128, axis=0)
    _, pm = welch(m, fs=25, nperseg=256, noverlap=128, axis=0)
    band = (f >= 1.2) & (f <= 1.9)
    ao = np.sqrt(po[band].sum(axis=0) * (f[1] - f[0]))
    am = np.sqrt(pm[band].sum(axis=0) * (f[1] - f[0]))
    amplitude = np.log(np.maximum(am, 0.001) / np.maximum(ao, 0.001)) * np.sqrt(0.4 / 3)
    peak = np.flatnonzero(band)[np.argmax(po[band, 2])]
    _, co = csd(a[:, 2], a[:, 1], fs=25, nperseg=256, noverlap=128)
    _, cm = csd(m[:, 2], m[:, 1], fs=25, nperseg=256, noverlap=128)
    coherent = abs(co[peak]) ** 2 / max(po[peak, 1] * po[peak, 2], 1e-12) >= 0.5
    phase = np.array(
        [
            np.cos(np.angle(cm[peak])) - np.cos(np.angle(co[peak])),
            np.sin(np.angle(cm[peak])) - np.sin(np.angle(co[peak])),
        ]
    ) * np.sqrt(0.1)
    if not coherent:
        phase *= 0
    r = np.r_[rate, acc, amplitude, phase]
    assert np.isclose(r @ r, prior.metrics(obs, pred)["score"], rtol=1e-9)
    return r


def timing(obs, pred, commands):
    a, m, u = obs[50:-10], pred[50:-10], commands[50:-10]
    result = {}
    for name, command, axis in [
        ("differential_to_yaw", u[:, 2] - u[:, 1], 2),
        ("common_to_pitch", u[:, 1:3].mean(axis=1), 1),
    ]:
        f, pu = welch(command, fs=25, nperseg=256, noverlap=128)
        _, pa = welch(a[:, axis], fs=25, nperseg=256, noverlap=128)
        _, pm = welch(m[:, axis], fs=25, nperseg=256, noverlap=128)
        _, ca = csd(command, a[:, axis], fs=25, nperseg=256, noverlap=128)
        _, cm = csd(command, m[:, axis], fs=25, nperseg=256, noverlap=128)
        coa = abs(ca) ** 2 / np.maximum(pu * pa, 1e-20)
        com = abs(cm) ** 2 / np.maximum(pu * pm, 1e-20)
        band = (f >= 1.2) & (f <= 1.9)
        peak = np.flatnonzero(band)[np.argmax(pa[band])]
        result[name] = {
            "frequency_hz": float(f[peak]),
            "command_std_mm": float(command.std()),
            "observed_coherence": float(coa[peak]),
            "model_coherence": float(com[peak]),
            "observed_phase_deg": float(np.rad2deg(np.angle(ca[peak]))),
            "model_phase_deg": float(np.rad2deg(np.angle(cm[peak]))),
            "phase_error_deg": float(
                np.rad2deg(np.angle(cm[peak] * np.conj(ca[peak])))
            ),
            "interpretable": bool(
                command.std() > 0.1 and coa[peak] >= 0.5 and com[peak] >= 0.5
            ),
        }
    return result


def worker(job):
    start, _theta, _alpha, _label = job
    original = prior.metrics
    captured = {}

    def capture(obs, pred):
        captured.update(obs=obs, pred=pred)
        return original(obs, pred)

    prior.metrics = capture
    prior.OUT = OUT
    try:
        row = prior.evaluate(job)
    finally:
        prior.metrics = original
    if not row["finite"]:
        raise RuntimeError(row)
    r = residual(captured["obs"], captured["pred"])
    a, _, i, _ = prior.prior.load("trout_lake", start)
    commands = np.column_stack([a["U"][i : i + 501, 0], a["brake_mm"][i : i + 501]])
    row["command_response"] = timing(captured["obs"], captured["pred"], commands)
    return row, r


class BudgetEnd(Exception):
    pass


class Objective:
    def __init__(self, pool, budget=None):
        self.pool, self.budget = pool, budget
        self.trace, self.cache = [], {}

    def __call__(self, z):
        key = tuple(z)
        if key in self.cache:
            return self.cache[key]
        if self.budget is not None and len(self.trace) >= self.budget:
            raise BudgetEnd()
        theta = LOW + WIDTH * np.array(z)
        t = time.monotonic()
        batch = list(self.pool.map(worker, [(s, theta, 0.35, "") for s in prior.TRAIN]))
        rows, residuals = zip(*batch)
        r = np.r_[
            np.concatenate(residuals) / np.sqrt(len(batch)),
            np.log(theta[:5]) * np.sqrt(0.005 / 5),
        ]
        self.trace.append(
            {
                "theta": theta.tolist(),
                "objective": float(r @ r),
                "fit": float(np.mean([v["score"] for v in rows])),
                "seconds": time.monotonic() - t,
                "rate_rmse": np.mean([v["rate_rmse"] for v in rows], axis=0).tolist(),
                "local": float(np.mean([v["local"] for v in rows])),
            }
        )
        self.cache[key] = r
        if len(self.trace) % 20 == 0:
            print(
                "evaluations",
                len(self.trace),
                "best",
                min(v["objective"] for v in self.trace),
                flush=True,
            )
        return r


def jacobian(fun, z, h):
    cols = []
    for j in range(6):
        lo, hi = np.array(z).copy(), np.array(z).copy()
        lo[j] = max(0, z[j] - h)
        hi[j] = min(1, z[j] + h)
        cols.append((fun(hi) - fun(lo)) / (hi[j] - lo[j]))
    return np.column_stack(cols)


def sensitivity(pool):
    selected = json.loads((OLD / "constrained-selection.json").read_text())["theta"]
    reports = {}
    for name, theta in [("production", prior.DEFAULT), ("constrained", selected)]:
        fun = Objective(pool)
        z = (np.array(theta) - LOW) / WIDTH
        r = fun(z)
        jac = jacobian(fun, z, 0.001)
        fine = jacobian(fun, z, 0.0005)
        # Exclude prior residuals from identifiability so regularization cannot create information.
        j = jac[:-5]
        _, sv, vt = np.linalg.svd(j, full_matrices=False)
        norms = np.linalg.norm(j, axis=0)
        corr = (j.T @ j) / np.maximum(np.outer(norms, norms), 1e-30)
        reports[name] = {
            "theta": theta,
            "objective": float(r @ r),
            "singular_values": sv.tolist(),
            "condition_number": float(sv[0] / sv[-1]) if sv[-1] > 1e-15 else None,
            "inactive_columns": np.flatnonzero(norms < 1e-12).tolist(),
            "sensitivity_norms": norms.tolist(),
            "cosine_similarity": corr.tolist(),
            "weakest_direction": vt[-1].tolist(),
            "relative_jacobian_step_change": (
                np.linalg.norm(j - fine[:-5], axis=0) / np.maximum(norms, 1e-15)
            ).tolist(),
            "objective_gradient": (2 * jac.T @ r).tolist(),
            "evaluation_count": len(fun.trace),
            "interpretation": "Normalized full-bound parameter units; one-sided at bounds. Local conditional sensitivity, not confidence intervals.",
        }
        np.savez_compressed(
            OUT / f"sensitivity-{name}.npz",
            jacobian=j,
            fine_jacobian=fine[:-5],
            residual=r,
        )
        print("sensitivity", name, reports[name], flush=True)
    save("sensitivity.json", reports)


def main():
    save(
        "protocol.json",
        {
            "names": prior.NAMES,
            "bounds": prior.BOUNDS,
            "training": prior.TRAIN,
            "manual_check": prior.MANUAL_CHECK,
            "nonmanual_check": prior.AUTO_CHECK,
            "starts": STARTS,
            "budget_per_start": BUDGET,
            "methods": ["Powell", "least_squares"],
            "budget_unit": "One unique parameter vector evaluated on all five 20 s training windows; numerical Jacobian probes count. Equal ceilings, early convergence allowed.",
            "residual": "Exact previous 0.4 local + 0.4 amplitude + 0.2 circular phase, plus .005 mean squared log parameter regularization; linear least squares loss.",
            "normalization": "All parameters transformed to [0,1] by exploratory bounds.",
            "jacobian": "Bound-aware central differences, h=.001 of parameter range; sensitivity repeats at .0005.",
            "selection": "Lowest training objective among finite evaluations, including derivative probes; freeze all runs before validation.",
            "limitations": "Same flight and previously reused validation windows; no new independent flight evidence. Timing is descriptive command/output association, not causal plant identification.",
            "deployment": False,
        },
    )
    with ProcessPoolExecutor(max_workers=4) as pool:
        sensitivity(pool)
        selections = {}
        for method in ["Powell", "least_squares"]:
            for seed, theta in enumerate(STARTS):
                tag = f"{method}-{seed}"
                fun = Objective(pool, BUDGET)
                z = (np.array(theta) - LOW) / WIDTH
                status = {}
                try:
                    if method == "Powell":
                        opt = minimize(
                            lambda x, fun=fun: float(fun(x) @ fun(x)),
                            z,
                            method="Powell",
                            bounds=[(0, 1)] * 6,
                            options={"maxfev": BUDGET, "xtol": 0.005, "ftol": 0.001},
                        )
                    else:
                        opt = least_squares(
                            fun,
                            z,
                            jac=lambda x, fun=fun: jacobian(fun, x, 0.001),
                            bounds=(np.zeros(6), np.ones(6)),
                            ftol=0.001,
                            xtol=0.001,
                            gtol=1e-6,
                            max_nfev=BUDGET,
                        )
                    status = {
                        "success": bool(opt.success),
                        "message": str(opt.message),
                        "reported_nfev": int(opt.nfev),
                    }
                except BudgetEnd:
                    status = {
                        "success": False,
                        "message": "Equal simulation evaluation budget exhausted",
                    }
                best = min(fun.trace, key=lambda v: v["objective"])
                selections[tag] = dict(
                    best,
                    status=status,
                    evaluations=len(fun.trace),
                    wall_seconds=sum(v["seconds"] for v in fun.trace),
                )
                save(f"trace-{tag}.json", fun.trace)
                save("selection.json", selections)
                print("selected", tag, selections[tag], flush=True)
        # Selection file is complete before any validation evaluation.
        all_rows = []
        families = {
            "baseline": prior.DEFAULT,
            "previous_constrained": json.loads(
                (OLD / "constrained-selection.json").read_text()
            )["theta"],
            **{k: v["theta"] for k, v in selections.items()},
        }
        for family, theta in families.items():
            for alpha in [0.25, 0.35, 0.45]:
                jobs = [
                    (s, theta, alpha, family)
                    for s in prior.MANUAL_CHECK + prior.AUTO_CHECK
                ]
                all_rows.extend(
                    dict(row, family=family) for row, _ in pool.map(worker, jobs)
                )
            save("validation.json", all_rows)
            print("validated", family, flush=True)
        summary = {}
        for family in families:
            if family == "baseline":
                continue
            summary[family] = {}
            for group, starts in [
                ("manual", prior.MANUAL_CHECK),
                ("nonmanual", prior.AUTO_CHECK),
            ]:
                base = [
                    r
                    for r in all_rows
                    if r["family"] == "baseline" and r["start"] in starts
                ]
                new = [
                    r
                    for r in all_rows
                    if r["family"] == family and r["start"] in starts
                ]
                summary[family][group] = prior.summarize(base, new)
        save("summary.json", summary)
    hashes = {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__), OLD / "run.py", OLD / "constrained-selection.json"]
    }
    save("source-hashes.json", hashes)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
