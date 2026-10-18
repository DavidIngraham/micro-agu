"""Final numerical, identifiability and episode-level audits."""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import json
from pathlib import Path

import numpy as np
from microagu_sim.sysid.data import load_session
from microagu_sim.sysid.diagnostics import sensitivity
from microagu_sim.sysid.metrics import metrics
from microagu_sim.sysid.model import replay

OUT = Path(__file__).resolve().parent


def main():
    _, segments, _ = load_session(OUT / "session.json")
    choices = json.loads((OUT / "frozen-selection.json").read_text())
    results = {}
    for name, selected in choices.items():
        print("Audit", name, flush=True)
        diag = sensitivity(segments, selected["theta"], selected["nuisance"])
        numerical = []
        for s in segments:
            if s.name not in ("tl-380", "tl-2760"):
                continue
            p, _, _ = replay(s, selected["theta"], substeps=32)
            q, _, _ = replay(s, selected["theta"], substeps=64)
            obs = np.rad2deg(s.observation[:, 3:6])
            numerical.append(
                {
                    "name": s.name,
                    "max_rate_rmse_difference": float(
                        np.max(
                            np.abs(
                                np.array(metrics(obs, p)["rate_rmse"])
                                - metrics(obs, q)["rate_rmse"]
                            )
                        )
                    ),
                }
            )
        results[name] = {"sensitivity": diag, "integration": numerical}
    (OUT / "audit.json").write_text(
        json.dumps(results, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
