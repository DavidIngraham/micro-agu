"""Keep severity, position, recovery and ownership as separate metrics."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent


def assess(run):
    rows = run["samples"]
    if not rows:
        return dict(id=run["spec"]["id"], complete=False, failure=run["failure"])
    time = np.array([r["time"] for r in rows])
    state = np.array([r["state"] for r in rows])
    angles = Rotation.from_quat(state[:, 3:7]).as_euler("xyz", degrees=True)
    active = np.array([r["active"] for r in rows], bool)
    result = dict(id=run["spec"]["id"], complete=run["complete"],
                  failure=run["failure"], kind=run["spec"]["kind"],
                  duration=float(time[-1]), minimum_height=float(min(-state[:, 2])),
                  peak_roll_deg=float(max(abs(angles[:, 0]))),
                  peak_pitch_deg=float(max(abs(angles[:, 1]))),
                  peak_sink=float(max(state[:, 9])),
                  peak_speed=float(max(np.linalg.norm(state[:, 7:10], axis=1))),
                  lqi_active_fraction=float(active.mean()),
                  late_active_fraction=float(active[time > time[-1]-5].mean()))
    contact = run["touchdown"]
    if contact:
        row = contact["incoming"] or contact["contact"]
        y = np.array(row["state"])
        delta = y[:2] - [run["target"]["north"], run["target"]["east"]]
        h = run["target"]["heading"]
        heading = np.array([np.cos(h), np.sin(h)])
        result.update(touchdown_sink=float(y[9]),
                      touchdown_ground_speed=float(np.linalg.norm(y[7:9])),
                      spot_error=float(np.linalg.norm(delta)),
                      along_error=float(delta@heading),
                      cross_error=float(delta@np.array([-heading[1], heading[0]])),
                      touchdown_angles=Rotation.from_quat(y[3:7]).as_euler("xyz", degrees=True).tolist(),
                      incoming_height=float(-y[2]),
                      contact_definition=contact["definition"])
    if run["spec"]["kind"] == "recovery":
        # Require five continuous seconds, below 0.2 m/s sink/climb and within
        # one metre of initial altitude; report timeout instead of forced success.
        ok = (abs(state[:, 2]-state[0, 2]) < 1) & (abs(state[:, 9]) < .2) & (time > 18)
        since = None
        recovered = None
        for t, valid in zip(time, ok):
            since = (t if since is None else since) if valid else None
            if since is not None and t-since >= 5:
                recovered = float(t)
                break
        result.update(recovery_time=recovered,
                      final_height_error=float(state[-1, 2]-state[0, 2]))
        result["altitude_loss"] = float(max(state[:, 2])-state[0, 2])
    return result


def main():
    results = [assess(json.loads(p.read_text())) for p in sorted((HERE/"ap-runs").glob("*.json"))]
    (HERE/"ap-summary.json").write_text(json.dumps(results, indent=2)+"\n")
    for result in results:
        print(json.dumps(result))


if __name__ == "__main__":
    main()
