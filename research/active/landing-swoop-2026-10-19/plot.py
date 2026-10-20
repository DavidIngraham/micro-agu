"""Publication-sized static figures; run after assess.py."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
rows = json.loads((HERE/"native-summary.json").read_text())
rows += [r for r in json.loads((HERE/"refine-summary.json").read_text())
         if r["spec"]["id"].startswith("fine-")]
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
for brake in [0, 18, 36, 54, 68]:
    selected = [r for r in rows if r["spec"]["id"].startswith(("flare-", "fine-"))
                and r["spec"]["flare_brake"] == brake]
    selected.sort(key=lambda r: r["spec"]["flare_height"])
    for ax, key in zip(axes, ["sink", "ground_speed"]):
        ax.plot([r["spec"]["flare_height"] for r in selected],
                [r["metrics"]["contact"][key] for r in selected],
                "o-", label=f"{brake} mm")
        ax.set_xlabel("Common-brake trigger height (m AGL)")
        ax.grid(alpha=.2)
axes[0].set_ylabel("Incoming sink speed (m/s)")
axes[1].set_ylabel("Incoming ground speed (m/s)")
axes[0].legend(title="Added common brake", loc="upper left", fontsize=9)
fig.suptitle("A narrow flare window in the current model — calm, truth-guided, no AP")
fig.savefig(HERE/"flare-window.png", dpi=180)
plt.close(fig)

fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
for name in ["flare-0-0", "flare-1-54", "flare-2-54"]:
    r = json.loads((HERE/"native-runs"/f"{name}.json").read_text())
    x = np.array(r["samples"])
    time = x[:, 0]-r["metrics"]["contact"]["time"]
    keep = time >= -4
    axes[0].plot(time[keep], x[keep, 10], label=name)
    axes[1].plot(time[keep], np.degrees(x[keep, 30]), label=name)
axes[0].set_ylabel("Sink speed (m/s)")
axes[1].set_ylabel("Payload pitch (degrees)")
axes[1].set_xlabel("Seconds relative to first contact")
axes[0].legend()
for ax in axes:
    ax.grid(alpha=.2)
fig.suptitle("Braking changes both drag and the transient pitch response")
fig.savefig(HERE/"flare-transient.png", dpi=180)
plt.close(fig)

ap = json.loads((HERE/"ap-summary.json").read_text())
fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), constrained_layout=True)
for scheme, color in [("baseline", "#4477aa"), ("lqi", "#cc6677")]:
    selected = [next(r for r in ap if r["id"] == f"land-{scheme}-{w}")
                for w in ["calm", "head", "cross"]]
    offset = -.17 if scheme == "baseline" else .17
    for ax, key in zip(axes, ["along_error", "touchdown_sink"]):
        ax.bar(np.arange(3)+offset, [r[key] for r in selected], .34, color=color,
               label="Classical" if scheme == "baseline" else "LQI approach → classical LAND")
        ax.set_xticks(np.arange(3), ["Calm", "Headwind", "Crosswind"])
        ax.grid(axis="y", alpha=.2)
axes[0].set_ylabel("Along-track spot error (m; + overshoot)")
axes[1].set_ylabel("Incoming sink (m/s)")
axes[0].legend(fontsize=9)
fig.suptitle("Actual ArduPlane LAND missions — first-contact approach, no brake-flare override")
fig.savefig(HERE/"ap-landings.png", dpi=180)
plt.close(fig)

fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
for scheme in ["baseline", "lqi"]:
    for rate, style in [(0, "-"), (25, "--")]:
        r = json.loads((HERE/"ap-runs"/f"matched-input-{scheme}-{rate}.json").read_text())
        t = np.array([x["time"] for x in r["samples"]])
        y = np.array([x["state"] for x in r["samples"]])
        for ax, values in zip(axes, [-y[:, 2], y[:, 9]]):
            ax.plot(t, values, style, label=f"{scheme}, {rate}°/s request")
axes[0].set_ylabel("Payload height (m AGL)")
axes[1].set_ylabel("Sink speed (m/s)")
axes[1].set_xlabel("Seconds after descent command")
axes[0].legend(ncol=2, fontsize=9)
for ax in axes:
    ax.axvline(8, color="gray", alpha=.4)
    ax.axvline(18, color="gray", alpha=.4)
    ax.grid(alpha=.2)
fig.suptitle("Altitude-only recovery precursor — matched 1 m/s descent / 0.75 m/s climb requests")
fig.savefig(HERE/"recovery.png", dpi=180)
