"""Post-selection mechanism ablations. Never select a replacement from checks."""

import json
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import run


def main():
    theta = json.loads((run.OUT / "selection.json").read_text())["theta"]
    variants = {"selected": theta}
    for name, indices in [
        ("original_rotor", [0]),
        ("original_suspension", [1, 2, 3]),
        ("original_pull", [4]),
        ("no_lag", [5]),
        ("suspension_only", [0, 4, 5]),
    ]:
        x = theta.copy()
        for j in indices:
            x[j] = run.DEFAULT[j]
        variants[name] = x
    results = {}
    records = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        for name, x in variants.items():
            r = list(
                pool.map(
                    run.evaluate,
                    [
                        (t, x, 0.35, "")
                        for t in run.TRAIN + run.MANUAL_CHECK + run.AUTO_CHECK
                    ],
                )
            )
            records.extend(dict(row, variant=name) for row in r)
            results[name] = {
                label: float(
                    np.mean([row["score"] for row in r if row["start"] in starts])
                )
                for label, starts in [
                    ("training", run.TRAIN),
                    ("manual", run.MANUAL_CHECK),
                    ("automatic", run.AUTO_CHECK),
                ]
            }
    run.save("ablations.json", dict(summary=results, records=records))
    print(results)


if __name__ == "__main__":
    main()
