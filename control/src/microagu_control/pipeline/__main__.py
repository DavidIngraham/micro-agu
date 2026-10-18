"""CLI-only pipeline: init, run, verify-gains. Every run has immutable inputs."""

import argparse
import json
import re
import shutil
from importlib.metadata import version
from pathlib import Path

import numpy as np

from microagu_sim import _native, compat

from ..sysid.aircraft import snapshot
from .artifacts import (
    context_id,
    decode_gains,
    export_header,
    gain_bytes,
    read_header,
    sha,
    write_json,
)
from .control import linearize, screen, synthesize


def run(protocol_path, output):
    if output.exists():
        raise ValueError("Output exists; choose a new run directory")
    cfg = json.loads(protocol_path.read_text())
    if cfg["schema"] != 1:
        raise ValueError("Unknown protocol schema")
    for group in ["models", "candidates"]:
        names = [item["name"] for item in cfg[group]]
        if (
            len(names) != len(set(names))
            or not names
            or any(not re.fullmatch(r"[a-z][a-z0-9_]*", n) for n in names)
        ):
            raise ValueError("Invalid or duplicate artifact name")
    if "reference" in [item["name"] for item in cfg["candidates"]]:
        raise ValueError("Reference name is reserved")
    if any(m["base"] not in ["fit", "production"] for m in cfg["models"]):
        raise ValueError("Unknown model base")
    root = protocol_path.resolve().parent
    aircraft = root / cfg["aircraft"]
    reference_path = root / cfg["reference"]
    fit_path = root / cfg["fit"]
    profile = json.loads(aircraft.read_text())
    reference = read_header(reference_path)
    fitted = json.loads(fit_path.read_text())
    if fitted.get("aircraft_profile_sha256") is not None and fitted[
        "aircraft_profile_sha256"
    ] != sha(aircraft):
        raise ValueError("Fit belongs to a different measured aircraft profile")
    theta = fitted["selection"]["theta"]
    if not fitted.get("training_episodes"):
        raise ValueError("Fit artifact must identify training episodes")
    if cfg["sensing"] != "ekf_air_velocity_no_canopy_truth":
        raise ValueError("Unsupported sensing contract")
    if reference["TRIM"][1] <= 1e-5:
        raise ValueError("This local screen requires an interior brake trim")
    # The adapter does not silently support changed servo geometry.
    strokes = np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1
    if not np.allclose(
        [
            profile["actuators"]["left_travel_mm"],
            profile["actuators"]["right_travel_mm"],
        ],
        strokes,
    ):
        raise ValueError(
            "Changed actuator geometry requires an updated control adapter"
        )
    output.mkdir(parents=True)
    frozen = output / "inputs"
    frozen.mkdir()
    inputs = {}
    for name, path in [
        ("protocol.json", protocol_path),
        ("aircraft.json", aircraft),
        ("reference.h", reference_path),
        ("fit.json", fit_path),
    ]:
        shutil.copyfile(path, frozen / name)
        inputs[name] = sha(path)
    manifest = {
        "schema": 1,
        "inputs": inputs,
        "source_hashes": {p.name: sha(p) for p in Path(__file__).parent.glob("*.py")},
        "native_sha256": sha(_native.__file__),
        "physics_inputs": {
            p.name: sha(p)
            for p in Path(compat.__file__).parent.iterdir()
            if p.suffix in {".py", ".json", ".npz"}
        },
        "versions": {
            name: version(name) for name in ["microagu-sim", "numpy", "scipy"]
        },
        "context_id": context_id(reference),
        "sensing": cfg["sensing"],
        "deployment_authorized": False,
        "limitation": "Local linear observer-in-loop screen only. No EKF, saturation, antiwindup, rate limiting, turn scheduling or power claim.",
    }
    write_json(output / "manifest.json", manifest)
    models = cfg["models"]
    plants = {}
    for model in models:
        vector = theta.copy() if model["base"] == "fit" else [1, 1, 1, 1, 1, 0]
        for key, value in model.get("override", {}).items():
            vector[int(key)] = value
        print("Linearizing " + model["name"], flush=True)
        plant = linearize(profile, vector, float(reference["TRIM"][1]))
        plants[model["name"]] = plant
        np.savez(output / (model["name"] + ".npz"), **plant)
    controllers = {"reference": reference}
    for candidate in cfg["candidates"]:
        controllers[candidate["name"]] = synthesize(
            plants[candidate["model"]], reference, candidate["weights"]
        )
    results = {}
    for name, controller in controllers.items():
        folder = output / name
        folder.mkdir()
        raw = gain_bytes(controller)
        decode_gains(raw, reference)
        (folder / "lqi-gains.bin").write_bytes(raw)
        (folder / "AP_CustomControl_LQI_gains.h").write_text(
            export_header(reference_path, controller)
        )
        results[name] = {
            model: screen(plant, controller) for model, plant in plants.items()
        }
        write_json(folder / "screen.json", results[name])
    eligible = [
        name
        for name in controllers
        if name != "reference" and all(v["stable"] for v in results[name].values())
    ]
    decision = {
        "eligible_for_nonlinear_validation": eligible,
        "deploy": False,
        "reason": "Native EKF/SITL stability, tracking, actuator and power comparison required"
        if eligible
        else "No candidate passed stability across the frozen model ensemble",
        "summary": {
            name: {
                model: {k: v for k, v in values.items() if k != "poles"}
                for model, values in cases.items()
            }
            for name, cases in results.items()
        },
        "unquantified_measurements": [
            k for k, v in profile["measurements"].items() if v["uncertainty"] is None
        ],
    }
    write_json(output / "decision.json", decision)
    write_json(
        output / "artifact-hashes.json",
        {str(p.relative_to(output)): sha(p) for p in output.rglob("*") if p.is_file()},
    )
    print(json.dumps(decision, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="action", required=True)
    init = sub.add_parser("init")
    init.add_argument("output", type=Path)
    r = sub.add_parser("run")
    r.add_argument("protocol", type=Path)
    r.add_argument("output", type=Path)
    v = sub.add_parser("verify-gains")
    v.add_argument("file", type=Path)
    v.add_argument("reference", type=Path)
    c = sub.add_parser("campaign")
    c.add_argument("run", type=Path)
    c.add_argument("protocol", type=Path)
    c.add_argument("firmware", type=Path)
    c.add_argument("viewer", type=Path)
    c.add_argument("output", type=Path)
    c.add_argument("--workers", type=int, default=2)
    args = p.parse_args()
    if args.action == "init":
        if args.output.exists():
            p.error("Output already exists")
        write_json(args.output, snapshot())
    elif args.action == "run":
        run(args.protocol, args.output)
    elif args.action == "campaign":
        from .campaign import campaign

        if not 1 <= args.workers <= 8:
            p.error("workers must be 1..8")
        campaign(
            args.run,
            args.protocol,
            args.firmware,
            args.viewer,
            args.output,
            args.workers,
        )
    else:
        decode_gains(args.file.read_bytes(), read_header(args.reference))
        print("Gain file valid and compatible")


if __name__ == "__main__":
    main()
