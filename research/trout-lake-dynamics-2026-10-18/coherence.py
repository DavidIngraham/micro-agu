"""Post-comparison diagnostic for the observed yaw spectral peak."""

import json
from pathlib import Path
import numpy as np
from scipy.signal import welch, csd

OUT = Path(__file__).resolve().parent
PRIOR = OUT.parent / "trout-lake-fidelity-2026-10-18"
names = [
    "differential_brake",
    "common_brake",
    "throttle",
    "roll_rate",
    "pitch_rate",
    "yaw_rate",
]
px = []
xy = []
for start in [t + d for t in [1440, 2100, 2760] for d in [0, 30, 60]]:
    data = np.load(PRIOR / f"baseline-trout_lake-{start}-0.35.npz")
    u = data["commands"]
    r = np.rad2deg(data["observed"][:, 3:6])
    x = np.column_stack([u[:, 2] - u[:, 1], u[:, 1:].mean(axis=1), u[:, 0], r])[50:-50]
    f, p = welch(x, fs=25, nperseg=256, noverlap=128, axis=0)
    px.append(p)
    xy.append(
        np.column_stack(
            [
                csd(x[:, j], x[:, -1], fs=25, nperseg=256, noverlap=128)[1]
                for j in range(6)
            ]
        )
    )
p = np.mean(px, axis=0)
c = np.mean(xy, axis=0)
coh = abs(c) ** 2 / np.maximum(p * p[:, -1, None], 1e-12)
i = np.argmin(abs(f - 1.5625))
result = {
    "frequency_hz": float(f[i]),
    "yaw_coherence": dict(zip(names, coh[i].tolist())),
    "interpretation": "Pooled closed-loop association, not causal attribution; missing disturbances or sensor processing can affect spectra.",
}
(OUT / "peak-coherence.json").write_text(json.dumps(result, indent=2) + "\n")
print(result)
