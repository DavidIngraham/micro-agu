"""Validated prepared-log input contract and episode-isolated session loading."""

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Segment:
    name: str
    episode: str
    role: str
    flight: str
    time: np.ndarray
    observation: np.ndarray
    commands: np.ndarray
    force_initial: np.ndarray
    wind: np.ndarray
    voltage: float
    rpm: float
    wind_sigma: float = 2.0
    initial_state: np.ndarray | None = None
    aircraft_profile: dict | None = None

    def __post_init__(self):
        n = len(self.time)
        if (
            self.aircraft_profile is not None
            and self.aircraft_profile.get("flight") != self.flight
        ):
            raise ValueError("Aircraft profile does not match flight configuration")
        if (
            self.role not in ("train", "validation")
            or not self.episode
            or not self.name
        ):
            raise ValueError("Named segment and episode with explicit role required")
        if self.flight not in ("trout_lake", "hood_river"):
            raise ValueError("Unsupported aircraft profile")
        if n < 321 or not np.allclose(np.diff(self.time), 0.04, atol=1e-5, rtol=0):
            raise ValueError(
                "At least 12.8 seconds of uniformly sampled 25 Hz data required"
            )
        if self.observation.shape != (n, 9) or self.commands.shape != (n, 3):
            raise ValueError(
                "Expected Nx9 observations and Nx3 throttle/brake-mm commands"
            )
        if self.wind.shape != (3,) or self.force_initial.shape != (3,):
            raise ValueError("Wind and initial specific force must be three-vectors")
        if not all(
            np.isfinite(x).all()
            for x in [
                self.time,
                self.observation,
                self.commands,
                self.wind,
                self.force_initial,
                self.voltage,
                self.rpm,
                self.wind_sigma,
            ]
        ):
            raise ValueError("Nonfinite data")
        if self.voltage <= 0 or self.rpm < 0 or self.wind_sigma <= 0:
            raise ValueError("Invalid voltage, RPM or wind uncertainty")
        if self.initial_state is not None and (
            self.initial_state.shape != (26,)
            or not np.isfinite(self.initial_state).all()
        ):
            raise ValueError("Invalid synthetic initial state")


def validate_split(segments):
    if len({s.name for s in segments}) != len(segments):
        raise ValueError("Duplicate segment names")
    roles = {
        role: {(s.flight, s.episode) for s in segments if s.role == role}
        for role in ["train", "validation"]
    }
    if not roles["train"]:
        raise ValueError("No training episodes")
    if roles["train"] & roles["validation"]:
        raise ValueError("Training and validation share an episode")
    for i, a in enumerate(segments):
        for b in segments[i + 1 :]:
            if (
                a.flight == b.flight
                and a.role != b.role
                and max(a.time[0], b.time[0]) <= min(a.time[-1], b.time[-1])
            ):
                raise ValueError("Training and validation overlap in time")


def load_session(path):
    path = Path(path)
    cfg = json.loads(path.read_text())
    if cfg.get("schema_version") != 1:
        raise ValueError("Unsupported session schema")
    segments = []
    hashes = {}
    arrays = {}
    aircraft_profile = None
    if cfg.get("aircraft_profile"):
        profile_path = (path.parent / cfg["aircraft_profile"]).resolve()
        aircraft_profile = json.loads(profile_path.read_text())
        hashes["aircraft_profile"] = hashlib.sha256(
            profile_path.read_bytes()
        ).hexdigest()
        from .aircraft import make_aircraft

        make_aircraft(aircraft_profile, [1, 1, 1, 1, 1, 0])
    for spec in cfg["segments"]:
        pair = []
        for key in ["causal_input", "propulsion_input"]:
            file = (path.parent / spec[key]).resolve()
            if file not in arrays:
                hashes[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
                with np.load(file, allow_pickle=False) as raw:
                    arrays[file] = {k: raw[k] for k in raw.files}
            pair.append(arrays[file])
        a, b = pair
        if a["t"].shape != b["t"].shape or not np.allclose(
            a["t"], b["t"], atol=1e-6, rtol=0
        ):
            raise ValueError("Propulsion and navigation clocks differ")
        i = int(np.argmin(abs(a["t"] - spec["start_s"])))
        if abs(a["t"][i] - spec["start_s"]) > 0.021:
            raise ValueError("Requested start outside log")
        n = round(spec.get("duration_s", 20) / 0.04) + 1
        sl = slice(i, i + n)
        if len(a["t"][sl]) != n or not a["valid"][sl].all():
            raise ValueError("Incomplete or invalid measurement window")
        wind = a["wind"][i].copy()
        sigma = float(spec.get("wind_sigma_m_s", 2.0))
        # Ground wind is an uncertain slow mean, never a sample-for-sample gust input.
        if "ground_wind" in cfg:
            w = cfg["ground_wind"]
            file = (path.parent / w["path"]).resolve()
            hashes[str(file)] = hashlib.sha256(file.read_bytes()).hexdigest()
            raw = np.genfromtxt(file, delimiter=",", names=True)
            t = np.atleast_1d(raw["time_s"]) + float(w["time_offset_s"])
            north, east = (
                np.atleast_1d(raw["north_m_s"]),
                np.atleast_1d(raw["east_m_s"]),
            )
            if (
                len(t) < 2
                or not np.all(np.diff(t) > 0)
                or not np.isfinite(np.r_[t, north, east]).all()
            ):
                raise ValueError("Invalid ground wind timestamps or values")
            if t[0] > a["t"][i] or t[-1] < a["t"][i + n - 1]:
                raise ValueError("Ground wind does not cover segment")
            inside = (t >= a["t"][i]) & (t <= a["t"][i + n - 1])
            covered = np.r_[a["t"][i], t[inside], a["t"][i + n - 1]]
            if np.max(np.diff(covered)) > float(w.get("max_gap_s", 3)):
                raise ValueError("Ground wind gap exceeds configured tolerance")
            wind[:2] = [np.interp(a["t"][sl], t, c).mean() for c in [north, east]]
            sigma = float(w["airborne_difference_sigma_m_s"])
        segments.append(
            Segment(
                spec["name"],
                spec["episode"],
                spec["role"],
                spec["flight"],
                a["t"][sl].copy(),
                a["Y"][sl].copy(),
                np.column_stack([a["U"][sl, 0], a["brake_mm"][sl]]),
                a["force"][max(0, i - 10) : i + 1].mean(axis=0),
                wind,
                float(b["voltage"][i]),
                float(b["rpm"][i]),
                sigma,
                aircraft_profile=aircraft_profile,
            )
        )
    validate_split(segments)
    return cfg, segments, hashes
