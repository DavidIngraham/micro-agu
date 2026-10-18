"""Render selected replay results without refitting."""

from pathlib import Path
import json
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
colors = {"baseline": "#84929f", "zero-only": "#de9e36", "coupled": "#3976be"}
for family, color in colors.items():
    d = np.load(ROOT / f"{family}-hood_river-438-0.35.npz")
    time = d["time"] - 438
    for ax, col in zip(axes, [0, 2, 4]):
        ax.plot(time, d["filtered_model"][:, col], color=color, label=family)
        if family == "baseline":
            ax.plot(
                time,
                d["filtered_observed"][:, col],
                color="#17252d",
                label="Flight",
                linewidth=2,
            )
for ax, label in zip(
    axes, ["Bank (deg)", "Unwrapped course change (deg)", "Climb (m/s)"]
):
    ax.set_ylabel(label)
    ax.grid(alpha=0.2)
    ax.axvspan(9, 18, color="#607d8b", alpha=0.08)
axes[0].legend(ncol=4)
axes[-1].set_xlabel("Seconds from replay start")
fig.suptitle(
    "Hood River powered turn: better bank, unresolved turning and energy\n0.7 Hz low-pass; shaded interval is 447–456 s; no model promoted"
)
fig.tight_layout()
fig.savefig(ROOT / "powered-turn.png", dpi=170)
summary = json.loads((ROOT / "summary.json").read_text())
fig, ax = plt.subplots(figsize=(10, 4.5))
x = np.arange(8)
for k, (name, color) in enumerate(list(colors.items())[1:]):
    ax.bar(
        x + (k - 0.5) * 0.36,
        100 * (np.array(summary[name]["window_ratios"]) - 1),
        0.36,
        label=name,
        color=color,
    )
ax.axhline(0, color="black", linewidth=0.7)
ax.axhline(15, color="#b74b4b", linestyle="--", label="Worst-window limit")
ax.set_xticks(
    x,
    [
        "HR 360",
        "HR 600",
        "HR 720",
        "HR 1020",
        "TL 840",
        "TL 1350",
        "TL 2400",
        "TL 2910",
    ],
)
ax.set_ylabel("Check score change (%) — lower is better")
ax.legend()
ax.grid(axis="y", alpha=0.2)
ax.set_title("A better Hood River fit does not transfer across both flights")
fig.tight_layout()
fig.savefig(ROOT / "check-windows.png", dpi=170)
