"""Second-stage training-only constrained selection; validation explicitly reused."""

import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import minimize
import run


def main():
    first = json.loads((run.OUT / "selection.json").read_text())["theta"]
    protocol = dict(
        reason="First fit improves amplitude/phase but regresses local waveform error. Constrain training errors before repeating validation.",
        validation="Reused after first-stage inspection, not independent evidence",
        training_local_limit=1.03,
        training_rate_channel_limit=1.03,
        optimizer="Powell two starts, 180 evaluations each; select lowest original training score among feasible evaluated points; default baseline included",
        bounds=run.BOUNDS,
    )
    run.save("constrained-protocol.json", protocol)
    trace = []
    status = []
    with ProcessPoolExecutor(max_workers=4) as pool:

        def batch(theta, starts=run.TRAIN, alpha=0.35, label=""):
            return list(
                pool.map(run.evaluate, [(s, theta, alpha, label) for s in starts])
            )

        base = batch(run.DEFAULT)

        def objective(theta):
            rows = batch(theta)
            if not all(r["finite"] for r in rows):
                return 1e6
            fit = float(np.mean([r["score"] for r in rows]))
            local = float(
                np.mean([r["local"] / b["local"] for r, b in zip(rows, base)])
            )
            ratios = np.mean([r["rate_rmse"] for r in rows], axis=0) / np.mean(
                [r["rate_rmse"] for r in base], axis=0
            )
            excess = np.maximum(np.r_[local, ratios] - 1.02, 0)
            penalty = 150 * float(excess @ excess)
            trace.append(
                dict(
                    theta=list(theta),
                    fit=fit,
                    local_ratio=local,
                    rate_ratios=ratios.tolist(),
                    feasible=bool(local <= 1.03 and max(ratios) <= 1.03),
                    objective=fit + penalty,
                )
            )
            if len(trace) % 20 == 0:
                print(
                    "constrained evaluation",
                    len(trace),
                    fit,
                    local,
                    ratios.tolist(),
                    flush=True,
                )
            return fit + penalty

        objective(run.DEFAULT)
        for seed, x in enumerate([run.DEFAULT, first]):
            opt = minimize(
                objective,
                x,
                method="Powell",
                bounds=run.BOUNDS,
                options=dict(maxfev=180, xtol=0.005, ftol=0.001),
            )
            status.append(
                dict(
                    seed=seed,
                    success=bool(opt.success),
                    message=str(opt.message),
                    nfev=int(opt.nfev),
                )
            )
            run.save("constrained-optimization.json", trace)
            run.save("constrained-optimizer-status.json", status)
        choice = min([r for r in trace if r["feasible"]], key=lambda r: r["fit"])
        run.save("constrained-selection.json", choice)
        print("Constrained selection", choice, flush=True)
        rows = []
        for alpha in [0.25, 0.35, 0.45]:
            rows.extend(
                dict(r, family="candidate")
                for r in batch(
                    choice["theta"],
                    run.MANUAL_CHECK + run.AUTO_CHECK,
                    alpha,
                    "constrained",
                )
            )
        basechecks = [
            r
            for r in json.loads((run.OUT / "validation.json").read_text())
            if r["family"] == "baseline"
        ]
        run.save("constrained-validation.json", rows)
    summary = {}
    for name, starts in [("manual", run.MANUAL_CHECK), ("automatic", run.AUTO_CHECK)]:
        summary[name] = run.summarize(
            [r for r in basechecks if r["start"] in starts],
            [r for r in rows if r["start"] in starts],
        )
    checks = {}
    for name, threshold in [("manual", 0.9), ("automatic", 0.95)]:
        r = summary[name]
        checks[name] = bool(
            r["mean_score_ratio"] <= threshold
            and max(r["rate_error_ratios"]) <= 1.05
            and r["mean_local_ratio"] <= 1.05
            and r["mean_amplitude_error_ratio"] <= 0.95
            and r["phase_error_deg"] <= r["baseline_phase_error_deg"] + 5
        )
    summary.update(
        gates=checks,
        passed=all(checks.values()),
        validation_reused=True,
        production_changed=False,
    )
    run.save("constrained-summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
