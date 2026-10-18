"""Identification contracts: leakage, broken clocks, wind coverage, and no deployment."""

import json
from dataclasses import replace

import numpy as np
import pytest
from microagu_control.sysid.data import Segment, load_session, validate_split
from microagu_control.sysid.fit import FitOptions, fit
from microagu_control.sysid.metrics import (
    legacy_residual,
    metrics,
    residual_diagnostics,
    response_residual,
)


def segment(name="a", episode="a", role="train", start=0.0):
    t = start + np.arange(501) * 0.04
    obs = np.zeros((501, 9))
    obs[:, 6] = 5
    commands = np.zeros((501, 3))
    return Segment(
        name,
        episode,
        role,
        "trout_lake",
        t,
        obs,
        commands,
        np.array([0.0, 0.0, -9.81]),
        np.zeros(3),
        22.0,
        1000.0,
    )


def test_episode_leakage_and_overlap():
    a = segment()
    with pytest.raises(ValueError, match="episode"):
        validate_split([a, segment("b", "a", "validation", 30)])
    with pytest.raises(ValueError, match="overlap"):
        validate_split([a, segment("b", "b", "validation", 10)])
    validate_split([a, segment("b", "b", "validation", 30)])


def test_invalid_measurements():
    a = segment()
    with pytest.raises(ValueError, match="uniformly"):
        replace(a, time=a.time + np.where(np.arange(501) > 30, 0.01, 0))
    with pytest.raises(ValueError, match="Nonfinite"):
        replace(a, observation=np.full((501, 9), np.nan))


def test_legacy_objective_equivalence_and_mean_error():
    t = np.arange(501) * 0.04
    actual = np.column_stack(
        [3 * np.sin(9 * t), 10 * np.sin(10 * t), 20 * np.cos(10 * t)]
    )
    predicted = actual * 0.7 + np.array([2, 3, 4])
    r = legacy_residual(actual, predicted)
    assert r @ r == pytest.approx(metrics(actual, predicted)["score"], rel=1e-12)
    assert np.allclose(response_residual(actual, actual), 0)
    assert np.linalg.norm(response_residual(actual, actual + 2)) > 0


def test_residual_lag_direction():
    rng = np.random.default_rng(6)
    command = rng.normal(size=501)
    predicted = np.zeros((501, 3))
    predicted[4:, 2] = command[:-4]
    commands = np.column_stack([np.zeros(501), np.zeros(501), command])
    r = residual_diagnostics(np.zeros_like(predicted), predicted, commands)[
        "differential_yaw"
    ]
    assert np.argmax(r["past_command_vs_error"]) == 3
    assert r["past_command_vs_error"][3] == pytest.approx(1)


def prepared(tmp_path):
    s = segment()
    np.savez(
        tmp_path / "input.npz",
        t=s.time,
        Y=s.observation,
        U=s.commands,
        brake_mm=s.commands[:, 1:],
        valid=np.ones(501, dtype=bool),
        wind=np.zeros((501, 3)),
        force=np.tile(s.force_initial, (501, 1)),
    )
    np.savez(
        tmp_path / "power.npz",
        t=s.time,
        voltage=np.full(501, 22.0),
        rpm=np.full(501, 1000.0),
    )
    cfg = {
        "schema_version": 1,
        "segments": [
            {
                "name": "a",
                "episode": "a",
                "role": "train",
                "flight": "trout_lake",
                "start_s": 0,
                "duration_s": 20,
                "causal_input": "input.npz",
                "propulsion_input": "power.npz",
            }
        ],
    }
    path = tmp_path / "session.json"
    path.write_text(json.dumps(cfg))
    return path, cfg


def test_ground_wind_offset_and_gaps(tmp_path):
    path, cfg = prepared(tmp_path)
    (tmp_path / "wind.csv").write_text(
        "time_s,north_m_s,east_m_s\n" + "".join(f"{i - 100},2,-1\n" for i in range(21))
    )
    cfg["ground_wind"] = {
        "path": "wind.csv",
        "time_offset_s": 100,
        "airborne_difference_sigma_m_s": 1.5,
    }
    path.write_text(json.dumps(cfg))
    _, data, _ = load_session(path)
    np.testing.assert_allclose(data[0].wind, [2, -1, 0])
    assert data[0].wind_sigma == 1.5
    (tmp_path / "wind.csv").write_text(
        "time_s,north_m_s,east_m_s\n-100,2,-1\n-80,2,-1\n"
    )
    with pytest.raises(ValueError, match="gap"):
        load_session(path)


def test_missing_samples_rejected(tmp_path):
    path, _ = prepared(tmp_path)
    with np.load(tmp_path / "input.npz") as src:
        a = dict(src)
    a["valid"][100] = False
    np.savez(tmp_path / "input.npz", **a)
    with pytest.raises(ValueError, match="invalid measurement"):
        load_session(path)


def test_fit_never_consumes_validation(monkeypatch):
    from microagu_control.sysid import fit as module

    class Pool:
        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def map(self, fn, jobs):
            return map(fn, jobs)

    names = []

    def job(args):
        s, theta, _, _ = args
        names.append(s.name)
        assert s.role == "train"
        return np.array([theta[0] - 1.1, theta[1] - 0.9])

    monkeypatch.setattr(module, "ProcessPoolExecutor", Pool)
    monkeypatch.setattr(module, "_job", job)
    a = segment()
    b = segment("reserved", "b", "validation", 30)
    result = fit([a, b], FitOptions(max_replays=20), starts=[[1, 1, 1, 1, 1, 0.04]])
    assert set(names) == {"a"}
    assert result["training_names"] == ["a"]
    assert result["production_changed"] is False


def test_identifiability_flags_unexcited_parameters(monkeypatch):
    from microagu_control.sysid import diagnostics

    class Pool:
        def __init__(self, **kwargs):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def map(self, fn, jobs):
            return map(fn, jobs)

    def job(args):
        _, theta, _, _ = args
        return np.array([theta[0], theta[1], 0.0, 0.0, 0.0, 0.0, 0.0])

    monkeypatch.setattr(diagnostics, "ProcessPoolExecutor", Pool)
    monkeypatch.setattr(diagnostics, "_job", job)
    result = diagnostics.sensitivity([segment()], [1, 1, 1, 1, 1, 0.04])
    assert result["inactive_columns"] == [2, 3, 4, 5]
    assert result["condition_number"] is None


def test_short_window_coherence_does_not_imply_supported_phase():
    from microagu_control.sysid.metrics import timing

    t = np.arange(501) * 0.04
    sine = np.sin(2 * np.pi * 1.6 * t)
    rates = np.column_stack([sine, sine, sine])
    commands = np.column_stack([np.zeros(len(t)), np.zeros(len(t)), 10 * sine])
    result = timing(rates, rates, commands)["differential_to_yaw"]
    assert result["welch_averages"] == 2
    assert result["coherence_screen_pass"]
    assert not result["interpretable"]


def test_relabeling_session_cannot_validate_training_episode():
    from microagu_control.sysid.validation import validate

    with pytest.raises(ValueError, match="Frozen fit trained"):
        validate(
            [segment(), segment("reserved", "b", "validation", 30)],
            [1, 1, 1, 1, 1, 0.04],
            training_episodes=["a", "b"],
        )


def test_measured_profile_reaches_identification(tmp_path):
    from microagu_control.sysid.aircraft import snapshot
    from microagu_control.sysid.model import configured_simulator

    profile = snapshot()
    profile["measurements"]["Ixx"]["value"] *= 1.2
    (tmp_path / "aircraft.json").write_text(json.dumps(profile))
    path, cfg = prepared(tmp_path)
    cfg["aircraft_profile"] = "aircraft.json"
    path.write_text(json.dumps(cfg))
    _, segments, hashes = load_session(path)
    assert "aircraft_profile" in hashes
    sim = configured_simulator(segments[0])
    assert sim.parameters["Ixx"] == profile["measurements"]["Ixx"]["value"]
    with pytest.raises(ValueError, match="does not match"):
        replace(segments[0], flight="hood_river")


def test_explicit_profile_preserves_replay():
    from microagu_control.sysid.aircraft import snapshot
    from microagu_control.sysid.model import replay

    a = segment()
    baseline = replay(a)
    explicit = replay(replace(a, aircraft_profile=snapshot()))
    for before, after in zip(baseline, explicit):
        np.testing.assert_array_equal(before, after)
