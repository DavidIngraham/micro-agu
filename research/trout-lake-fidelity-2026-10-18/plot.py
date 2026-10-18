"""Visualize frozen validation results."""

import json
from pathlib import Path
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parent
summary = json.loads((OUT / "summary.json").read_text())
fig, ax = plt.subplots(figsize=(10, 4.8))
x = np.arange(9)
for j, (family, color) in enumerate([("zero-only", "#de9e36"), ("physics", "#3976be")]):
    ax.bar(
        x + (j - 0.5) * 0.36,
        100 * (np.array(summary[family]["window_ratios"]) - 1),
        0.36,
        label=family,
        color=color,
    )
ax.axhline(0, color="black", linewidth=0.7)
ax.axhline(15, color="#b74b4b", linestyle="--", label="Worst-window limit")
ax.set_xticks(x, [str(t + d) for t in [1440, 2100, 2760] for d in [0, 30, 60]])
ax.set_xlabel("Trout Lake validation replay start (s)")
ax.set_ylabel("Score change (%) — lower is better")
ax.set_title("Trout Lake only calibration: reserved-block validation")
ax.legend()
ax.grid(axis="y", alpha=0.2)
fig.tight_layout()
fig.savefig(OUT / "validation.png", dpi=170)
family = summary["selected_family"]
fig, axes = plt.subplots(3, 3, figsize=(13, 9), sharex="col")
for j, start in enumerate([1440, 2100, 2760]):
    for name, color in [("baseline", "#84929f"), ("selected", "#3976be")]:
        d = np.load(OUT / f"long-{name}-trout_lake-{start}-0.35.npz")
        for ax, col in zip(axes[:, j], [0, 2, 4]):
            ax.plot(
                d["time"] - start, d["filtered_model"][:, col], label=name, color=color
            )
            if name == "baseline":
                ax.plot(
                    d["time"] - start,
                    d["filtered_observed"][:, col],
                    color="#17252d",
                    label="Flight",
                )
    axes[0, j].set_title(f"Block starting {start} s")
    axes[-1, j].set_xlabel("Elapsed seconds")
for ax, label in zip(
    axes[:, 0], ["Bank (deg)", "Unwrapped course change (deg)", "Climb (m/s)"]
):
    ax.set_ylabel(label)
for ax in axes.flat:
    ax.grid(alpha=0.2)
axes[0, 0].legend()
fig.suptitle(f"Continuous 90 s validation: selected {family} fit; 0.7 Hz low-pass")
fig.tight_layout()
fig.savefig(OUT / "long-replays.png", dpi=170)
