"""Reproduce the reserved powered-turn diagnostic from the recorded session."""
import json
from pathlib import Path
import numpy as np
from microagu_control.sysid.data import load_session
from microagu_control.sysid.model import DEFAULT, replay

study = Path(__file__).resolve().parent
_, segments, _ = load_session(study / "session.json")
segment = next(s for s in segments if s.name == "hr-438")
theta = json.loads((study / "fit/fit.json").read_text())["selection"]["theta"]
mask = (segment.time >= 447) & (segment.time <= 456)
observed = np.rad2deg(segment.observation[:, 3:6])
result = {
    "interval_seconds": [447, 456], "units": "deg/s",
    "observed_mean_rate": observed[mask].mean(0).tolist(),
    "observed_rate_std": observed[mask].std(0).tolist(), "rows": [],
}
arrays = {"time": segment.time, "observed_rates": observed}
for family, parameters in [("baseline", DEFAULT), ("fitted", theta)]:
    for alpha in (0.25, 0.35, 0.45):
        rates, _, _ = replay(segment, parameters, alpha=alpha)
        result["rows"].append({
            "family": family, "alpha": alpha,
            "mean_rate": rates[mask].mean(0).tolist(),
            "rate_std": rates[mask].std(0).tolist(),
            "rate_rmse": np.sqrt(np.mean((rates[mask]-observed[mask])**2, axis=0)).tolist(),
        })
        if alpha == 0.35:
            arrays[family] = rates
(study / "powered-turn.json").write_text(json.dumps(result, indent=2) + "\n")
np.savez_compressed(study / "powered-turn.npz", **arrays)
