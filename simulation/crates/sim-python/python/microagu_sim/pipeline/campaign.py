"""Run isolated WASM firmware with file-loaded gains; never overwrite packaged assets."""

import json
import os
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from microagu_sim.compat import dynamics

from ..sysid.aircraft import make_aircraft
from .artifacts import sha, verify_run, write_json
from .assess import assess, compare, power_table


def campaign(run, protocol, firmware, viewer, output, workers=2):
    if output.exists():
        raise ValueError("Output already exists")
    output = output.resolve()
    protocol = protocol.resolve()
    run = run.resolve()
    verify_run(run)
    firmware = firmware.resolve()
    viewer = viewer.resolve()
    cfg = json.loads(protocol.read_text())
    if cfg.get("schema") != 1 or cfg.get("phase") not in ["training", "validation"]:
        raise ValueError("Invalid campaign schema or phase")
    identifiers = [case["id"] for case in cfg["cases"]]
    if len(identifiers) != len(set(identifiers)) or any(
        not isinstance(i, int) or i < 0 for i in identifiers
    ):
        raise ValueError("Case IDs must be unique nonnegative integers")
    expected_gates = {
        "tracking_improvement",
        "task_max_regression",
        "motion_max_regression",
        "power_max_regression",
    }
    if set(cfg["gates"]) != expected_gates or any(
        not 0 <= v <= 1 for v in cfg["gates"].values()
    ):
        raise ValueError("Invalid comparison gates")
    frozen = json.loads((run / "decision.json").read_text())
    controllers = ["reference", "reference_repeat"] + frozen[
        "eligible_for_nonlinear_validation"
    ]
    if cfg.get("controllers"):
        if not set(cfg["controllers"]) <= set(controllers):
            raise ValueError("Controller failed local screen")
        controllers = cfg["controllers"]
    if "reference" not in controllers or "reference_repeat" not in controllers:
        raise ValueError("Paired reference and identical-gain repeat required")

    def gain_path(name):
        return (
            output
            / "gains"
            / (("reference" if name == "reference_repeat" else name) + ".bin")
        )

    for case in cfg["cases"]:
        if any(
            case.get("parameters", {}).get(k, 0) != 0
            for k in ["ARSPD_TYPE", "ARSPD_USE"]
        ):
            raise ValueError(
                "Campaign must preserve the frozen no-pitot sensing contract"
            )
    output.mkdir(parents=True)
    (output / "gains").mkdir()
    for name in controllers:
        source_name = "reference" if name == "reference_repeat" else name
        if not gain_path(name).exists():
            shutil.copyfile(run / source_name / "lqi-gains.bin", gain_path(name))
    runtime = output / "runtime"
    runtime.mkdir()
    for suffix in [".js", ".wasm"]:
        shutil.copyfile(firmware.with_suffix(suffix), runtime / ("arduplane" + suffix))
    write_json(runtime / "package.json", {"type": "module"})
    firmware = runtime / "arduplane.js"
    jobs = []
    model = json.loads((run / "inputs/protocol.json").read_text())
    fit = json.loads((run / "inputs/fit.json").read_text())
    modelmap = {m["name"]: m for m in model["models"]}
    profile = json.loads((run / "inputs/aircraft.json").read_text())
    base = make_aircraft(profile, [1, 1, 1, 1, 1, 0])
    physics = json.loads((viewer / "assets/runtime/highdof.json").read_text())
    keys = physics["parameter_keys"]
    missing = set(dynamics.KEYS) - set(keys)
    if (
        keys != list(dynamics.KEYS)[: len(keys)]
        or not missing <= {"brake_cp_shift", "brake_outboard_fraction"}
        or any(base.parameters[k] != 0 for k in missing)
    ):
        raise ValueError("Browser physics parameter schema mismatch")
    physics.update(
        parameters=dynamics.params(base.parameters)[: len(keys)].tolist(),
        mass=base.geometry["mass"],
        imu=base.geometry["imu"].tolist(),
        brake_rate_mm_s=profile["actuators"]["brake_rate_mm_s"],
        brake_offsets_m=[
            base.parameters["left_offset"],
            base.parameters["right_offset"],
        ],
    )
    write_json(output / "physics.json", physics)
    manifest = {
        "firmware": {p.name: sha(p) for p in [firmware, firmware.with_suffix(".wasm")]},
        "gain_files": {n: sha(gain_path(n)) for n in controllers},
        "protocol_sha256": sha(protocol),
        "harness_sha256": sha(viewer / "tests/integration/pipeline-case.ts"),
        "physics_sha256": sha(viewer / "generated/physics/sim_wasm_bg.wasm"),
        "config_sha256": sha(output / "physics.json"),
        "source_hashes": {p.name: sha(p) for p in Path(__file__).parent.glob("*.py")},
    }
    write_json(output / "manifest.json", manifest)
    write_json(output / "protocol.json", cfg)
    tablepath = viewer.parent / "simulation/crates/sim-core/src/propeller_data.rs"
    table = power_table(tablepath)
    for name in controllers:
        (output / name).mkdir()
        for case in cfg["cases"]:
            model = modelmap[case["model"]]
            theta = (
                list(fit["selection"]["theta"])
                if model["base"] == "fit"
                else [1, 1, 1, 1, 1, 0]
            )
            for k, v in model.get("override", {}).items():
                theta[int(k)] = v
            spec = case | {
                "firmware": str(firmware),
                "modelConfig": str(output / "physics.json"),
                "gains": str(gain_path(name)),
                "theta": theta,
                "parameters": {
                    "CP3_HRTC": 0.7,
                    "ARSPD_TYPE": 0,
                    "ARSPD_USE": 0,
                }
                | case.get("parameters", {}),
            }
            path = output / name / (str(case["id"]) + ".json")
            source = path.with_suffix(".case.json")
            write_json(source, spec)
            jobs.append((name, case["id"], source, path))

    def execute(job):
        name, case, source, path = job
        with path.with_suffix(".log").open("w") as log:
            try:
                rc = subprocess.run(
                    [
                        str(viewer / "node_modules/.bin/tsx"),
                        str(viewer / "tests/integration/pipeline-case.ts"),
                        str(source),
                        "lqi",
                        str(path),
                    ],
                    check=False,
                    cwd=viewer,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    timeout=420,
                    env=dict(os.environ),
                ).returncode
            except subprocess.TimeoutExpired:
                rc = 124
        result = (
            assess(path, table)
            if path.exists()
            else {"gates": ["Missing result / infrastructure failure"], "metrics": None}
        )
        result.update(controller=name, case=str(case), exit_code=rc)
        print(json.dumps(result), flush=True)
        return result

    with ThreadPoolExecutor(max_workers=workers) as pool:
        rows = list(pool.map(execute, jobs))
    write_json(output / "metrics.json", rows)
    write_json(output / "comparison.json", compare(rows, cfg["gates"]))
    return rows
