"""Post-selection ablations; these do not select or promote another model."""

import json
from concurrent.futures import ProcessPoolExecutor
import run


def main():
    selected = json.loads((run.OUT / "selection.json").read_text())["coupled"]["theta"]
    variants = {"coupled": selected}
    for name, index in [("no_cp", 4), ("no_outboard", 5), ("no_residual_yaw", 6)]:
        theta = selected.copy()
        theta[index] = 0.0
        variants[name] = theta
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = []
        for name, theta in variants.items():
            rows.extend(
                dict(r, variant=name)
                for r in pool.map(
                    run.evaluate,
                    [
                        (f, t, theta, 0.35, "")
                        for f, t in run.CHECK + [("hood_river", 438)]
                    ],
                )
            )
    (run.OUT / "ablations.json").write_text(json.dumps(rows, indent=2) + "\n")
    a = run.evaluate(("hood_river", 438, selected, 0.35, ""))
    b = run.evaluate(("hood_river", 438, selected, 0.35, ""), substeps=64)
    integration = {
        "substeps_32": a,
        "substeps_64": b,
        "max_metric_delta": max(
            abs(a["metrics"][k] - b["metrics"][k]) for k in a["metrics"]
        ),
    }
    (run.OUT / "integration-check.json").write_text(
        json.dumps(integration, indent=2) + "\n"
    )
    print(
        {
            name: {
                "mean_score": sum(
                    r["score"]
                    for r in rows
                    if r["variant"] == name and r["start"] != 438
                )
                / 8,
                "turn": next(
                    r["turn"]["model"]
                    for r in rows
                    if r["variant"] == name and r["start"] == 438
                ),
            }
            for name in variants
        }
    )


if __name__ == "__main__":
    main()
