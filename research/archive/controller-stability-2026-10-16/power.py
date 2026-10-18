"""Post-process retained campaign logs; reconstructed propeller power, not battery power."""
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE = ROOT / "simulation/crates/sim-core/src"


def table(name):
    text = (SOURCE / "propeller_data.rs").read_text()
    body = re.search(rf"{name}: \[f64; \d+\] = \[(.*?)\];", text, re.S)[1]
    return np.array([float(x.strip()) for x in body.split(",") if x.strip()])


RPM = table("RPM_GRID")
CP = table("CP_GRID").reshape(25, 401)


def shaft_power(throttle, axial):
    # Same steady motor map and Cp interpolation as the campaign Rust model.
    # Actual rotor RPM was not retained; its 17.56 ms lag cannot be recovered exactly
    # from 100 ms command samples. This is explicitly a quasi-steady approximation.
    rpm = 743.5778222336925 * 22.2 * np.maximum(throttle - 0.0005141016498671315, 0) ** 0.9086807307118114
    valid = (rpm >= RPM[0]) & (rpm < RPM[-1])
    i = np.clip(np.searchsorted(RPM, rpm, side="right") - 1, 0, 23)
    f = (rpm - RPM[i]) / (RPM[i + 1] - RPM[i])
    j = np.clip(np.maximum(axial, 0) / (np.maximum(rpm, 1e-9) / 60 * 0.2286), 0, 2) * 200
    k = np.minimum(j.astype(int), 399)
    g = j - k
    a = CP[i, k] * (1 - g) + CP[i, k + 1] * g
    b = CP[i + 1, k] * (1 - g) + CP[i + 1, k + 1] * g
    cp = np.where(valid, a * (1 - f) + b * f, 0)
    return 1.225 * (rpm / 60) ** 3 * 0.2286 ** 5 * cp


def metrics(rows):
    t = np.array([s["time"] for s in rows])
    y = np.array([s["truth"] for s in rows])
    wind = np.array([s["wind"] for s in rows])
    throttle = np.array([s["command"][0] for s in rows])
    air_velocity = y[:, 7:10] - wind
    axial = Rotation.from_quat(y[:, 3:7]).inv().apply(air_velocity)[:, 0]
    power = shaft_power(throttle, axial)
    duration = t[-1] - t[0]
    integral = lambda v: float(np.trapezoid(v, t))
    mean = lambda v: integral(v) / duration
    distance = integral(np.linalg.norm(y[:, 7:9], axis=1))
    brakes = np.array([s["brakes"] for s in rows]) / (np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1)
    return dict(duration_s=duration, mean_throttle=mean(throttle),
                estimated_shaft_W=mean(power), estimated_shaft_Wh=integral(power) / 3600,
                ground_path_m=distance, estimated_shaft_Wh_per_ground_km=integral(power) / 3.6 / distance,
                mean_ground_speed_m_s=distance / duration,
                mean_airspeed_m_s=mean(np.linalg.norm(air_velocity, axis=1)),
                delta_height_m=float(y[0, 2] - y[-1, 2]),
                delta_ground_speed_m_s=float(np.linalg.norm(y[-1, 7:10]) - np.linalg.norm(y[0, 7:10])),
                mean_common_brake_fraction=mean(brakes.mean(axis=1)))


def main():
    reports = []
    for path in sorted((HERE / "results").glob("*.json")):
        d = json.loads(path.read_text())
        assert d["complete"], f"Incomplete run: {path}"
        # Refuse to apply a changed propulsion model to archived results.
        for name in ["propulsion.rs", "propeller_data.rs"]:
            expected = d["build"]["physics_source"]["source_sha256"][f"crates/sim-core/src/{name}"]
            assert hashlib.sha256((SOURCE / name).read_bytes()).hexdigest() == expected
        r = dict(id=d["spec"]["id"], scheme=d["scheme"], mode=d["spec"]["mode"])
        for phase, lo, hi in [("disturbed", 20, 100), ("recovery", 100, 180),
                             ("late_calm", 150, 180), ("combined", 20, 180)]:
            r[phase] = metrics([s for s in d["samples"] if lo <= s["time"] < hi])
        reports.append(r)
    (HERE / "power-metrics.json").write_text(json.dumps(reports, indent=2) + "\n")
    lines = ["# Power and energy comparison", "",
             "These logs contain commands, actual brake travel, state and wind, but **no battery",
             "current or retained rotor RPM**. Watts/Wh below are quasi-steady propeller shaft",
             "power estimates from the existing 22.2 V throttle-to-RPM map and Cp table, using",
             "logged body-axis airspeed. Source hashes match the campaign firmware manifest.", "",
             "They are **not electrical power, battery energy or endurance predictions**.",
             "Motor/ESC losses, servo power and avionics power are not modeled. The motor's",
             "17.56 ms lag is omitted because commands were saved only every 100 ms.", "",
             "## Interpretation", "",
             "LQI is not consistently more energy-efficient in this campaign. In every AUTO",
             "pair, combined-window estimated shaft energy is higher; the median paired power",
             "increase is 11.5%, and estimated energy per traveled ground distance rises 18.4%.",
             "LQI uses substantially more common brake and flies more slowly in these runs.",
             "That is consistent with extra brake drag, but is not a controlled causal ablation.", "",
             "CRUISE is mixed: combined-window median paired power change is approximately zero",
             "(-0.3%), with individual changes from -19.0% to +35.8%. In the two sustained",
             "baseline-oscillation cases (01, 15), late-calm LQI shaft-power estimates are",
             "14–16% lower. Conversely, cases 03, 07 and 09 settle under LQI with roughly",
             "26–27% common brake and 3.87–4.00 m/s airspeed; they require 13–17% more shaft",
             "power than their baseline partners. Suppressing oscillation helps, but an inefficient",
             "brake/throttle equilibrium can erase that advantage.", "",
             "Windows: disturbed 20–100 s, late calm 150–180 s, combined 20–180 s after",
             "controller selection. Combined energy ratios closely follow mean-power ratios",
             "because the paired windows have the same duration. Settling and launch are excluded.", "",
             "All values are time-weighted. Group rows are medians across eight cases.", "",
             "| Window | Mode | Controller | Throttle % | Est. shaft W | Est. shaft Wh/km ground path | Ground speed m/s | Common brake % |",
             "|---|---|---|---:|---:|---:|---:|---:|"]
    for phase in ["disturbed", "late_calm", "combined"]:
        for mode in ["AUTO", "CRUISE"]:
            for scheme in ["baseline", "lqi"]:
                group = [r[phase] for r in reports if r["mode"] == mode and r["scheme"] == scheme]
                med = lambda k: float(np.median([r[k] for r in group]))
                lines.append(f"| {phase} | {mode} | {scheme} | {100*med('mean_throttle'):.2f} | "
                             f"{med('estimated_shaft_W'):.2f} | {med('estimated_shaft_Wh_per_ground_km'):.2f} | "
                             f"{med('mean_ground_speed_m_s'):.2f} | {100*med('mean_common_brake_fraction'):.2f} |")
    lines += ["", "## Paired relative changes: LQI vs baseline", "",
              "Ratios are calculated within each pair before taking the median and range.", ""]
    for phase in ["disturbed", "late_calm", "combined"]:
        for mode in ["AUTO", "CRUISE"]:
            for key in ["estimated_shaft_W", "estimated_shaft_Wh_per_ground_km"]:
                changes = []
                for b in reports:
                    if b["mode"] == mode and b["scheme"] == "baseline":
                        l = next(r for r in reports if r["id"] == b["id"] and r["scheme"] == "lqi")
                        changes.append(100 * (l[phase][key] / b[phase][key] - 1))
                lines.append(f"- {phase} {mode}, {key}: {np.median(changes):+.1f}% "
                             f"(range {min(changes):+.1f}% to {max(changes):+.1f}%).")
    lines += ["", "## CRUISE late-calm cases", "",
              "| Case | Controller | Est. shaft W | Est. Wh/km ground path | Height change m | Speed change m/s |",
              "|---|---|---:|---:|---:|---:|"]
    for r in reports:
        if r["mode"] == "CRUISE":
            m = r["late_calm"]
            lines.append(f"| {r['id']} | {r['scheme']} | {m['estimated_shaft_W']:.2f} | "
                         f"{m['estimated_shaft_Wh_per_ground_km']:.2f} | {m['delta_height_m']:.3f} | "
                         f"{m['delta_ground_speed_m_s']:.3f} |")
    lines += ["", "Ground-path distance includes all traveled distance, not useful mission progress.",
              "AUTO paths and target tracking differ between controllers. Disturbed flight also",
              "has unequal thermal encounters and changes in stored potential/kinetic energy.",
              "Late calm CRUISE is the cleaner comparison, but finite windows and oscillation",
              "phase still affect endpoint energy. No result here establishes electrical efficiency.", "",
              "Reproduce with a NumPy/SciPy environment: `python power.py`. No reruns or retuning.",
              "`test_power.py` verifies the reconstructed shaft-power calculation against the",
              "native Rust torque map across 42 throttle/axial-speed combinations; it passes.",
              "This verifies interpolation and units, not the omitted motor dynamics or electrical losses."]
    (HERE / "POWER.md").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
