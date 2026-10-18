"""Artifact compatibility, aircraft input mapping and observer/lag closure tests."""

import copy
import struct
import zlib
from pathlib import Path

import numpy as np
import pytest
from microagu_sim.pipeline.artifacts import SHAPES, context_id, decode_gains, gain_bytes
from microagu_sim.pipeline.control import closed_loop, screen
from microagu_sim.sysid.aircraft import make_aircraft, snapshot


def reference():
    z = {k: np.zeros(shape) for k, shape in SHAPES.items()}
    z["XS"][:] = 1
    z["US"][:] = 0.1
    z["K"][:, 16:] = np.eye(3)
    z["AW"] = np.eye(3) * 10
    z["A"] = np.eye(16) * 0.8
    return z


def test_file_roundtrip_and_context():
    z = reference()
    data = gain_bytes(z)
    k, aw = decode_gains(data, z)
    np.testing.assert_array_equal(k, z["K"])
    np.testing.assert_array_equal(aw, z["AW"])
    other = copy.deepcopy(z)
    other["A"][0, 0] += 0.1
    assert context_id(other) != context_id(z)
    with pytest.raises(ValueError):
        decode_gains(data, other)


@pytest.mark.parametrize("change", ["truncated", "extra", "crc", "nan", "antiwindup"])
def test_corrupt_files(change):
    z = reference()
    data = gain_bytes(z)
    if change == "truncated":
        data = data[:-1]
    if change == "extra":
        data += b"x"
    if change == "crc":
        data = data[:-1] + bytes([data[-1] ^ 1])
    if change in ["nan", "antiwindup"]:
        raw = bytearray(data[16:])
        struct.pack_into(
            "<f",
            raw,
            0 if change == "nan" else 57 * 4,
            float("nan") if change == "nan" else 0,
        )
        data = data[:12] + struct.pack("<I", zlib.crc32(raw)) + raw
    with pytest.raises(ValueError):
        decode_gains(data, z)


def test_physical_inputs_and_fitted_parameters_separate():
    profile = snapshot()
    original = copy.deepcopy(profile)
    sim = make_aircraft(profile, [1, 0.8, 0.3, 1.2, 1.1, 0.12])
    assert sim.parameters["joint_k_yaw"] == pytest.approx(
        profile["parameters"]["joint_k_yaw"] * 0.8
    )
    assert sim.geometry["mass"] == 1.758
    assert sim.parameters["yaw_payload_halfspacing"] == 0.066
    assert profile == original
    baseline = make_aircraft(profile, [1, 1, 1, 1, 1, 0])
    np.testing.assert_allclose(
        sim.controls(0.4, 20, 20)[1:], baseline.controls(0.4, 20, 20)[1:] * 1.1
    )
    profile["measurements"]["mass"]["value"] += 0.2
    with pytest.raises(ValueError, match="Masses"):
        make_aircraft(profile, [1, 1, 1, 1, 1, 0])


def test_lag_is_a_real_closed_loop_state():
    z = reference()
    z["B"][0, 1] = 0.2
    z["L"][0, 0] = 1
    z["K"][1, 0] = 0.5
    plant = {
        "A": z["A"],
        "B": z["B"],
        "H": np.zeros((3, 16)),
        "theta": np.array([1, 1, 1, 1, 1, 0.0]),
    }
    no_lag = closed_loop(plant, z)
    plant["theta"][5] = 0.12
    lag = closed_loop(plant, z)
    assert not np.allclose(lag, no_lag)
    assert lag[38, 38] > 0
    # A deliberately unstable plant is rejected even when the nominal controller is stable.
    plant["A"] = np.eye(16) * 1.1
    assert not screen(plant, z)["stable"]


def test_tampered_run_rejected(tmp_path):
    from microagu_sim.pipeline.artifacts import sha, verify_run, write_json

    file = tmp_path / "input.json"
    file.write_text("{}")
    write_json(tmp_path / "artifact-hashes.json", {"input.json": sha(file)})
    verify_run(tmp_path)
    file.write_text('{"changed": true}')
    with pytest.raises(ValueError, match="artifact changed"):
        verify_run(tmp_path)


def test_nonrepeatable_reference_blocks_promotion():
    from microagu_sim.pipeline.assess import compare

    metrics = {
        "climb": 2,
        "descent": 2,
        "vertical": 2,
        "crosswind": 2,
        "relative_pitch_rate_rms": 5,
        "relative_pitch_rate_peak": 50,
        "shaft_W": 75,
        "shaft_Wh_per_km": 5,
    }
    rows = [
        {"controller": "reference", "case": "a", "gates": [], "metrics": metrics},
        {
            "controller": "reference_repeat",
            "case": "a",
            "gates": [],
            "metrics": metrics | {"relative_pitch_rate_peak": 80},
        },
        {
            "controller": "candidate",
            "case": "a",
            "gates": [],
            "metrics": {k: v * 0.8 for k, v in metrics.items()},
        },
    ]
    result = compare(rows)
    assert not result["repeatability"]["passed"]
    assert result["selected"] is None
    rows[1]["metrics"] = metrics.copy()
    assert compare(rows)["selected"] == "candidate"


def test_campaign_freezes_gains_and_retains_failed_runs(tmp_path, monkeypatch):
    import json
    from types import SimpleNamespace

    from microagu_sim.compat import dynamics
    from microagu_sim.pipeline import campaign as module
    from microagu_sim.pipeline.artifacts import sha, write_json

    run = tmp_path / "run"
    (run / "inputs").mkdir(parents=True)
    (run / "reference").mkdir()
    write_json(run / "decision.json", {"eligible_for_nonlinear_validation": []})
    write_json(
        run / "inputs/protocol.json",
        {"models": [{"name": "production", "base": "production"}]},
    )
    write_json(run / "inputs/fit.json", {"selection": {"theta": [1, 1, 1, 1, 1, 0]}})
    write_json(run / "inputs/aircraft.json", snapshot())
    (run / "reference/lqi-gains.bin").write_bytes(gain_bytes(reference()))
    write_json(
        run / "artifact-hashes.json",
        {str(p.relative_to(run)): sha(p) for p in run.rglob("*") if p.is_file()},
    )
    viewer = tmp_path / "viewer"
    for folder in ["assets/runtime", "tests/integration", "generated/physics"]:
        (viewer / folder).mkdir(parents=True)
    write_json(
        viewer / "assets/runtime/highdof.json", {"parameter_keys": list(dynamics.KEYS)}
    )
    (viewer / "tests/integration/pipeline-case.ts").write_text("// harness")
    (viewer / "generated/physics/sim_wasm_bg.wasm").write_bytes(b"physics")
    firmware = tmp_path / "firmware.js"
    firmware.write_text("// firmware")
    firmware.with_suffix(".wasm").write_bytes(b"firmware")
    protocol = tmp_path / "protocol.json"
    write_json(
        protocol,
        {
            "schema": 1,
            "phase": "training",
            "cases": [{"id": 0, "model": "production"}],
            "gates": {
                "tracking_improvement": 0.05,
                "task_max_regression": 0.2,
                "motion_max_regression": 0.1,
                "power_max_regression": 0.05,
            },
        },
    )
    output = tmp_path / "output"
    calls = []

    def execute(args, **kwargs):
        spec = json.loads(Path(args[2]).read_text())
        assert Path(spec["gains"]).is_relative_to(output)
        assert (
            Path(spec["gains"]).read_bytes()
            == (run / "reference/lqi-gains.bin").read_bytes()
        )
        assert json.loads(Path(spec["modelConfig"]).read_text())["mass"] == 1.758
        Path(args[-1]).write_text("{}")
        calls.append(spec)
        return SimpleNamespace(returncode=1)

    monkeypatch.setattr(module.subprocess, "run", execute)
    monkeypatch.setattr(module, "power_table", lambda p: None)
    monkeypatch.setattr(
        module,
        "assess",
        lambda p, t: {"gates": ["intentional failed flight"], "metrics": None},
    )
    module.campaign(run, protocol, firmware, viewer, output)
    assert len(calls) == 2
    result = json.loads((output / "comparison.json").read_text())
    assert not result["repeatability"]["passed"]
    assert not result["deploy"]
    assert json.loads((viewer / "assets/runtime/highdof.json").read_text()) == {
        "parameter_keys": list(dynamics.KEYS)
    }
