# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=2,<3", "scipy>=1.14,<2", "matplotlib>=3.9,<4"]
# ///
"""Report truth-based motion, tracking and effort separately, retaining failed runs."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import detrend, welch
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent


def rms(x):
    return float(np.sqrt(np.mean(np.asarray(x) ** 2)))


def motion(rows):
    t = np.array([s["time"] for s in rows])
    state = np.array([s["truth"] for s in rows])
    rotation = Rotation.from_quat(state[:, 3:7])
    attitude = rotation.as_euler("xyz", degrees=True)
    rates = rotation.inv().apply(state[:, 10:13]) * 180 / np.pi
    return t, attitude, rates


def metrics(rows):
    if len(rows) < 20:
        return None
    t, angles, rates = motion(rows)
    commands = np.array([s["command"] for s in rows])
    commands[:, 1:] /= np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1
    # Uniform interpolation is explicit; truth is sampled at ~10 Hz, never rendered-frame rate.
    grid = np.arange(t[0], t[-1], 0.1)
    pitch = detrend(np.interp(grid, t, angles[:, 1]))
    freq, power = welch(pitch, fs=10, nperseg=min(256, len(pitch)))
    band = (freq >= 0.1) & (freq <= 2)
    return dict(pitch_ac_deg=rms(detrend(angles[:, 1])),
                pitch_peak_to_peak_deg=float(np.ptp(angles[:, 1])),
                roll_ac_deg=rms(detrend(angles[:, 0])),
                pitch_rate_deg_s=rms(rates[:, 1]), roll_rate_deg_s=rms(rates[:, 0]),
                yaw_rate_ac_deg_s=rms(detrend(rates[:, 2])),
                pitch_band_rms_deg=float(np.sqrt(np.trapezoid(power[band], freq[band]))),
                pitch_peak_hz=float(freq[band][np.argmax(power[band])]),
                altitude_rms_m=rms([s["altitude"] - s["targetAltitude"] for s in rows]),
                xtrack_rms_m=rms([s["crossTrack"] for s in rows]),
                brake_endpoint_fraction=float(np.mean(np.any((commands[:, 1:] < 0.01) |
                                                         (commands[:, 1:] > 0.99), axis=1))),
                throttle_endpoint_fraction=float(np.mean((commands[:, 0] < 0.01) |
                                                       (commands[:, 0] > 0.99))),
                command_travel_per_s=(np.abs(np.diff(commands, axis=0)).sum(axis=0) /
                                      (t[-1] - t[0])).tolist())


def main():
    reports = []
    runs = {}
    for path in sorted((HERE / "results").glob("*.json")):
        d = json.loads(path.read_text())
        rows = d["samples"]
        report = dict(id=d["spec"]["id"], mode=d["spec"]["mode"], scheme=d["scheme"],
                      complete=d["complete"], failure=d["failure"], samples=len(rows))
        for name, lo, hi in [("disturbed", 20, 100), ("early_recovery", 105, 135),
                             ("late_recovery", 150, 180)]:
            report[name] = metrics([s for s in rows if lo <= s["time"] < hi])
        report["final_mission"] = rows[-1]["mission"] if rows else None
        reports.append(report)
        runs[(report["id"], report["scheme"])] = d
    (HERE / "metrics.json").write_text(json.dumps(reports, indent=2) + "\n")
    lines = ["# Results", "", f"{len(reports)} runs, "
             f"{sum(r['complete'] for r in reports)} completed without a screening stop.", "",
             "Medians below exclude incomplete runs; failures are listed separately.", "",
             "| Task | Controller | Complete | Disturbed pitch AC RMS ° | Pitch rate RMS °/s | Altitude RMS m | Cross-track RMS m | Late calm pitch AC RMS ° |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for mode in ["AUTO", "CRUISE"]:
        for scheme in ["baseline", "lqi"]:
            group = [r for r in reports if r["mode"] == mode and r["scheme"] == scheme]
            good = [r for r in group if r["complete"]]
            values = []
            for phase, key in [("disturbed", "pitch_ac_deg"), ("disturbed", "pitch_rate_deg_s"),
                               ("disturbed", "altitude_rms_m"), ("disturbed", "xtrack_rms_m"),
                               ("late_recovery", "pitch_ac_deg")]:
                values.append(f"{np.median([r[phase][key] for r in good]):.3f}" if good else "n/a")
            lines.append(f"| {mode} | {scheme} | {len(good)}/{len(group)} | " + " | ".join(values) + " |")
    lines += ["", "## Lateral motion and command effort", "",
              "Medians during disturbances. AUTO includes commanded turns. Travel is total",
              "absolute command change per second, in normalized actuator fractions.", "",
              "| Task | Controller | Roll AC RMS ° | Roll-rate RMS °/s | Yaw-rate AC RMS °/s | Throttle travel/s | Left travel/s | Right travel/s |",
              "|---|---|---:|---:|---:|---:|---:|---:|"]
    for mode in ["AUTO", "CRUISE"]:
        for scheme in ["baseline", "lqi"]:
            group = [r["disturbed"] for r in reports if r["mode"] == mode and
                     r["scheme"] == scheme and r["complete"]]
            if group:
                values = [np.median([r[k] for r in group]) for k in
                          ["roll_ac_deg", "roll_rate_deg_s", "yaw_rate_ac_deg_s"]]
                values.extend(np.median([r["command_travel_per_s"] for r in group], axis=0))
                lines.append(f"| {mode} | {scheme} | " + " | ".join(f"{x:.3f}" for x in values) + " |")
    lines += ["", "## Failures", ""]
    if all(r["complete"] for r in reports):
        lines.append("None. Completion alone does not mean acceptable damping.")
    lines.extend(f"- Case {r['id']} {r['scheme']}: {r['failure']} ({r['samples']} samples)"
                 for r in reports if not r["complete"])
    lines += ["", "## Paired differences (LQI minus baseline)", "",
              "Only complete pairs. Bootstrap intervals resample scenarios within task; with eight",
              "or fewer pairs they are exploratory, conditional on these assumed ranges.", ""]
    rng = np.random.default_rng(20261016)
    for mode in ["AUTO", "CRUISE"]:
        for key in ["pitch_ac_deg", "pitch_rate_deg_s", "altitude_rms_m"]:
            diff = []
            for b in reports:
                if b["mode"] != mode or b["scheme"] != "baseline" or not b["complete"]:
                    continue
                l = next(r for r in reports if r["id"] == b["id"] and r["scheme"] == "lqi")
                if l["complete"]:
                    diff.append(l["disturbed"][key] - b["disturbed"][key])
            if diff:
                boots = np.median(rng.choice(diff, (10000, len(diff))), axis=1)
                low, high = np.quantile(boots, [0.025, 0.975])
                lines.append(f"- {mode} {key}: median {np.median(diff):.3f}, "
                             f"95% paired bootstrap [{low:.3f}, {high:.3f}], n={len(diff)}.")
    lines += ["", "## Individual CRUISE recovery", "",
              "Linear-detrended truth pitch over 105–135 s and 150–180 s. A ratio near one",
              "at appreciable amplitude suggests persistent motion; it is not a pole estimate.", "",
              "| Case | Controller | Early RMS ° | Late RMS ° | Late/early | Late spectral peak Hz |",
              "|---|---|---:|---:|---:|---:|"]
    for r in reports:
        if r["mode"] != "CRUISE" or not r["complete"]:
            continue
        early, late = r["early_recovery"], r["late_recovery"]
        lines.append(f"| {r['id']} | {r['scheme']} | {early['pitch_ac_deg']:.3f} | "
                     f"{late['pitch_ac_deg']:.3f} | "
                     f"{late['pitch_ac_deg'] / max(early['pitch_ac_deg'], 1e-9):.3f} | "
                     f"{late['pitch_peak_hz']:.3f} |")
    lines += ["", "Actuator endpoint occupancy and total travel are in metrics.json. A released",
              "brake is normal for the baseline mixer; endpoint occupancy is not by itself",
              "evidence of controller saturation. The study does not record unconstrained demand."]
    lines += ["", "## Normal-speed check", ""]
    for scheme in ["baseline", "lqi"]:
        path = HERE / "pacing-check" / f"{scheme}.json"
        if path.exists():
            d = json.loads(path.read_text())
            late = metrics([s for s in d["samples"] if 150 <= s["time"] < 180])
            if late:
                lines.append(f"- {scheme}: complete={d['complete']}, late pitch AC RMS "
                             f"{late['pitch_ac_deg']:.3f} deg, pitch rate RMS "
                             f"{late['pitch_rate_deg_s']:.3f} deg/s, peak "
                             f"{late['pitch_peak_hz']:.3f} Hz.")
    (HERE / "RESULTS.md").write_text("\n".join(lines) + "\n")
    # Every trajectory shown; no cherry-picked example.
    fig, axes = plt.subplots(4, 4, figsize=(16, 12), sharex=True)
    for i, ax in enumerate(axes.flat):
        for scheme, color in [("baseline", "#3575b5"), ("lqi", "#cf553b")]:
            d = runs.get((i, scheme))
            if not d or len(d["samples"]) < 2:
                continue
            t, angles, _ = motion(d["samples"])
            ax.plot(t, angles[:, 1], color=color, lw=0.8, label=scheme)
            ax.set_title(f"{i:02} {d['spec']['mode']}")
        ax.axvspan(20, 100, color="gray", alpha=0.12)
        ax.set_ylabel("Truth pitch (deg)")
        ax.set_xlabel("Seconds after selection")
        ax.grid(alpha=0.2)
    axes[0, 0].legend()
    fig.suptitle("All paired cases: shaded interval has wind, turbulence and thermal")
    fig.tight_layout()
    fig.savefig(HERE / "pitch-traces.png", dpi=150)
    plt.close(fig)
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    for scheme, color in [("baseline", "#3575b5"), ("lqi", "#cf553b")]:
        d = runs.get((1, scheme))
        if not d or not d["samples"]:
            continue
        rows = d["samples"]
        t, angles, rates = motion(rows)
        axes[0].plot(t, angles[:, 1], color=color, label=scheme)
        axes[1].plot(t, rates[:, 1], color=color)
        axes[2].plot(t, [s["altitude"] - s["targetAltitude"] for s in rows], color=color)
        axes[3].plot(t, [s["command"][0] for s in rows], color=color)
    for ax, ylabel in zip(axes, ["Pitch (deg)", "Pitch rate (deg/s)", "Altitude error (m)", "Throttle fraction"]):
        ax.set_ylabel(ylabel)
        ax.axvspan(20, 100, color="gray", alpha=0.12)
        ax.grid(alpha=0.2)
    axes[0].legend()
    axes[0].set_title("Nominal CRUISE: disturbances removed at 100 s")
    axes[-1].set_xlabel("Seconds after controller selection")
    fig.tight_layout()
    fig.savefig(HERE / "nominal-cruise.png", dpi=150)
    print("\n".join(lines))


if __name__ == "__main__":
    main()
