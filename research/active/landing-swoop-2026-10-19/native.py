"""First-contact flare and altitude-only brake-release recovery experiments.

Uses the selected browser parameter vector, Rust kernel and servo travel limit.
This is a truth-guided maneuver feasibility study, not an AP controller.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation
from microagu_sim import _native
from microagu_sim.compat.actuator import BrakeActuator

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CONFIG = ROOT / "aircraft/microagu/configurations/browser-physics.json"


def run(spec, dt=.00125):
    config = json.loads(CONFIG.read_text())
    kernel = _native.Kernel(config["parameters"], config["mass"])
    y = np.array(config["state"], float)
    rpm = config["rpm"]
    # Preserve joint geometry and use an established, fully inflated canopy.
    y[[2, 15]] += -spec.get("height", 20) - y[2]
    wind = np.array(spec.get("wind", [0, 0, 0]), float)
    y[7:10] += wind
    y[20:23] += wind
    actuator = BrakeActuator(config["brake_rate_mm_s"], config["command"][1:])
    env = np.array([2., 0., 1.])
    rows = []
    contact = None
    release_height = None
    recovered = None
    stable_s = 0.
    flare = False
    flare_time = None
    load = 1.
    perturbed = False
    # Start with ten seconds of power-off glide before defining the approach.
    # For swoop, establish braked glide, then release and reapply brakes.
    settle = 10.
    duration = 70. if spec["kind"] == "landing" else 35.
    for i in range(round(duration / dt)):
        t = i * dt
        if not perturbed and t >= settle and -y[2] <= 3:
            y[[9, 22]] += spec.get("sink_perturbation", 0.)
            perturbed = True
        height = -y[2]
        common = spec.get("approach_brake", 0.)
        if spec["kind"] == "landing":
            trigger = spec["flare_height"]
            if spec.get("guidance") == "sink":
                trigger += spec.get("lead_s", .4) * max(0., y[9])
            if t >= settle and height + spec.get("height_bias", 0.) <= trigger:
                flare = True
                if flare_time is None:
                    flare_time = t
            if flare:
                common = spec["flare_brake"]
        else:
            common = spec["prebrake"] if t < settle else (
                0. if t < settle + spec["release_s"] else spec["recovery_brake"])
            if t >= settle and release_height is None:
                release_height = height
        # Constant horizontal wind plus optional deterministic gust, reproducible
        # across maneuvers; explicitly not a Dryden realization.
        w = wind.copy()
        gust = spec.get("gust", 0.)
        w += gust * np.array([np.sin(1.7*t), .6*np.sin(2.3*t+.4), .4*np.sin(.9*t)])
        target = np.array(config["command"][1:]) + common
        applied = actuator.advance(np.clip(target, 0, 72), dt)
        u = [0., *np.maximum(0., applied / 1000 - config["brake_offsets_m"])]
        before = y.copy()
        result = np.array(kernel.surface_step(y, u, w, config["voltage"],
                                             rpm, dt, env, [.1, 1.2, 4.5, 3., .8, .5]))
        y, rpm, env = result[:26], result[26], result[27:]
        if env[0] != 3:
            load = float(np.linalg.norm((y[7:10]-before[7:10])/dt-[0, 0, 9.80665])/9.80665)
        euler = Rotation.from_quat(before[3:7]).as_euler("xyz")
        rel = (Rotation.from_quat(before[3:7]).inv() *
               Rotation.from_quat(before[16:20])).as_rotvec()
        if i % max(1, round(.05/dt)) == 0 or env[0] == 3:
            rows.append([t, *before, *applied, *euler, *rel, *w, load])
        if env[0] == 3:
            # Evaluate incoming velocity BEFORE the plastic impulse removes it.
            contact = dict(time=t+dt, position=before[:3].tolist(),
                           sink=float(before[9]), ground_speed=float(np.linalg.norm(before[7:9])),
                           roll_deg=float(np.degrees(euler[0])),
                           pitch_deg=float(np.degrees(euler[1])),
                           speed=float(np.linalg.norm(before[7:10])))
            break
        if spec["kind"] == "swoop" and t > settle + spec["release_s"]:
            # Recovery is a sustained modest descent, not zero vertical speed.
            ok = abs(y[9]) < 1.5 and np.linalg.norm(y[10:13]) < .15
            stable_s = stable_s + dt if ok else 0.
            if stable_s >= 2 and recovered is None:
                recovered = dict(time=t, height=-float(y[2]))
    rows = np.asarray(rows)
    selected = rows[rows[:, 0] >= settle]
    if len(selected) == 0:
        selected = rows
    metrics = dict(contact=contact, flare_time=flare_time,
                   preapproach_contact=contact is not None and contact["time"] < settle,
                   peak_air_speed=float(np.max(np.linalg.norm(selected[:, 8:11]-selected[:, 35:38], axis=1))),
                   peak_specific_force_g=float(np.max(selected[:, 38])),
                   peak_relative_pitch_deg=float(np.degrees(np.max(abs(selected[:, 33])))),
                   peak_sink=float(np.max(selected[:, 10])),
                   peak_roll_deg=float(np.degrees(np.max(abs(selected[:, 29])))),
                   peak_pitch_deg=float(np.degrees(np.max(abs(selected[:, 30])))),
                   release_height=release_height, recovered=recovered,
                   observation_height_loss=float(release_height-np.min(-selected[:, 3])) if release_height is not None else None,
                   loss_to_recovery=(release_height-recovered["height"]) if recovered else None)
    return dict(spec=spec, dt=dt, metrics=metrics, samples=rows.tolist())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()
    cases = []
    for h in ([1, 3] if args.quick else [0, .5, 1, 2, 3, 4]):
        for brake in ([0, 36] if args.quick else [0, 18, 36, 54, 68]):
            cases.append(dict(id=f"flare-{h}-{brake}", kind="landing", height=30,
                              flare_height=h, flare_brake=brake))
    for release in ([1, 3] if args.quick else [.5, 1, 2, 3]):
        for brake in ([36] if args.quick else [18, 36, 54]):
            cases.append(dict(id=f"recovery-{release}-{brake}", kind="swoop",
                              height=100, prebrake=36, release_s=release,
                              recovery_brake=brake))
    # Uncertainty/perturbation tests are separate from the calm calibration grid.
    for wind in ([-1.5, 0, 0], [1.5, 0, 0], [0, 1.5, 0]):
        for guidance in ("height", "sink"):
            cases.append(dict(id=f"wind-{wind[0]}-{wind[1]}-{guidance}",
                              kind="landing", height=30, wind=wind,
                              guidance=guidance, flare_height=1, flare_brake=36, gust=.2))
    out = HERE / "native-runs"
    out.mkdir(exist_ok=True)
    results = []
    for spec in cases:
        result = run(spec)
        (out / (spec["id"]+".json")).write_text(json.dumps(result))
        results.append({k:v for k,v in result.items() if k != "samples"})
        print(spec["id"], json.dumps(result["metrics"]), flush=True)
    (HERE / "native-summary.json").write_text(json.dumps(results, indent=2)+"\n")
    (HERE / "native-provenance.json").write_text(json.dumps(dict(
        config_sha256=hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        native_sha256=hashlib.sha256(Path(_native.__file__).read_bytes()).hexdigest(),
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        cases=cases), indent=2)+"\n")


if __name__ == "__main__":
    main()
