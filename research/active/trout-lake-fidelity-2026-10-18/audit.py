"""Post-selection duration and integration sensitivity; no refitting."""

import json
import hashlib
import numpy as np
import run

original_load = run.load


def longer_load(flight, start):
    a, b, i, _ = original_load(flight, start)
    sl = slice(i, i + 2251)
    assert len(a["t"][sl]) == 2251 and a["valid"][sl].all()
    return a, b, i, sl


def main():
    choice = json.loads((run.OUT / "selection.json").read_text())["primary"]
    rows = []
    run.load = longer_load
    for family, theta in [("baseline", run.DEFAULT), ("selected", choice["theta"])]:
        for start in run.CHECK_BLOCKS:
            for alpha in [0.25, 0.35, 0.45]:
                rows.append(
                    dict(
                        run.evaluate(
                            ("trout_lake", start, theta, alpha, "long-" + family)
                        ),
                        family=family,
                    )
                )
    run.save("long-replays.json", rows)
    base = {(r["start"], r["alpha"]): r for r in rows if r["family"] == "baseline"}
    ratios = [
        r["score"] / base[r["start"], r["alpha"]]["score"]
        for r in rows
        if r["family"] == "selected"
    ]
    run.load = original_load
    historical = []
    for family, theta in [("baseline", run.DEFAULT), ("selected", choice["theta"])]:
        for start in [840, 1350, 2400, 2910]:
            historical.append(
                dict(
                    run.evaluate(("trout_lake", start, theta, 0.35, "")), family=family
                )
            )
    run.save("historical-checks.json", historical)
    historical_base = {r["start"]: r for r in historical if r["family"] == "baseline"}
    historical_ratios = {
        r["start"]: r["score"] / historical_base[r["start"]]["score"]
        for r in historical
        if r["family"] == "selected"
    }
    a = run.evaluate(("trout_lake", run.CHECK_BLOCKS[0], choice["theta"], 0.35, ""))
    b = run.evaluate(
        ("trout_lake", run.CHECK_BLOCKS[0], choice["theta"], 0.35, ""), substeps=64
    )
    result = {
        "historical_window_ratios": historical_ratios,
        "long_mean_ratio": float(np.mean(ratios)),
        "long_worst_ratio": max(ratios),
        "long_ratios": ratios,
        "all_finite": all(r["finite"] for r in rows),
        "integration_max_metric_delta": max(
            abs(a["metrics"][k] - b["metrics"][k]) for k in a["metrics"]
        ),
    }
    run.save("audit.json", result)
    from microagu_sim.compat.profiles import make_simulator

    configuration = {}
    for flight in ["trout_lake", "hood_river"]:
        sim = make_simulator(flight, profile="as_built")
        configuration[flight] = {
            "mass_kg": sim.geometry["mass"],
            "payload_mass_kg": sim.geometry["payload_mass"],
            "inertia_kg_m2": sim.geometry["inertia"].tolist(),
            "hinge_m": sim.geometry["hinge"].tolist(),
            "parameters": sim.parameters,
        }
    run.save("configurations.json", configuration)
    sim = make_simulator("trout_lake", profile="as_built")
    theta = choice["theta"]
    sim.parameters["left_offset"] += theta[0] / 1000
    sim.parameters["right_offset"] += theta[1] / 1000
    sim.parameters["brake_cp_shift"] = theta[2]
    sim.parameters["Cndelta"] = theta[3]
    for key, value in zip(
        ["joint_k_roll", "joint_c_pitch", "joint_c_yaw", "CLbrake", "CDbrake"],
        theta[4:],
    ):
        sim.parameters[key] *= value
    run.save(
        "candidate-parameters.json",
        {
            "status": "provisional; short-window validation passes, long-horizon regression unresolved",
            "flight": "trout_lake",
            "mass_kg": sim.geometry["mass"],
            "riser_spacing_m": 0.132,
            "parameters": sim.parameters,
        },
    )
    environment = []
    for start in run.CHECK_BLOCKS:
        data, propulsion, index, _ = original_load("trout_lake", start)
        sl = slice(index, index + 2251)
        environment.append(
            dict(
                start=start,
                voltage_start=float(propulsion["voltage"][index]),
                voltage_range=[
                    float(propulsion["voltage"][sl].min()),
                    float(propulsion["voltage"][sl].max()),
                ],
                stored_replay_wind_start=data["wind"][index].tolist(),
                max_stored_replay_wind_change_mps=float(
                    np.linalg.norm(data["wind"][sl] - data["wind"][index], axis=1).max()
                ),
            )
        )
    run.save("environment-drift.json", environment)

    run.save(
        "source-hashes.json",
        {
            str(p.relative_to(run.ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [
                run.OUT / "run.py",
                run.OUT / "audit.py",
                run.ROOT / "simulation/crates/sim-core/src/dynamics.rs",
                run.ROOT / "simulation/crates/sim-core/src/config.rs",
            ]
        },
    )
    print(result)


if __name__ == "__main__":
    main()
