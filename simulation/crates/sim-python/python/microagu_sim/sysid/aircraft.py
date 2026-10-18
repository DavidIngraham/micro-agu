"""Measured inputs are explicit, versioned and kept separate from fitted coefficients."""

import copy

import numpy as np

from microagu_sim.compat.profiles import make_simulator
from microagu_sim.sysid.model import BOUNDS, NAMES

MEASUREMENTS = {
    "mass": ("geometry", "mass", "kg", "Flight configuration total mass"),
    "payload_mass": (
        "geometry",
        "payload_mass",
        "kg",
        "CAD-informed ballast configuration",
    ),
    "canopy_mass": (
        "parameter",
        "canopy_mass",
        "kg",
        "Canopy mass in aircraft profile",
    ),
    "Ixx": ("parameter", "Ixx", "kg m^2", "CAD estimate"),
    "Iyy": ("parameter", "Iyy", "kg m^2", "CAD estimate"),
    "Izz": ("parameter", "Izz", "kg m^2", "CAD estimate"),
    "yaw_payload_halfspacing": (
        "parameter",
        "yaw_payload_halfspacing",
        "m",
        "As-built 132 mm riser spacing / 2",
    ),
    "hinge_x": ("parameter", "hinge_x", "m", "CAD estimate"),
    "hinge_y": ("parameter", "hinge_y", "m", "CAD estimate"),
    "hinge_z": ("parameter", "hinge_z", "m", "CAD estimate"),
}


def snapshot():
    sim = make_simulator("trout_lake", profile="as_built")
    measurements = {}
    for name, (target, key, units, source) in MEASUREMENTS.items():
        value = (sim.geometry if target == "geometry" else sim.parameters)[key]
        measurements[name] = {
            "value": float(value),
            "units": units,
            "source": source,
            "uncertainty": None,
            "fixed": True,
        }
    return {
        "schema": 1,
        "flight": "trout_lake",
        "profile": "as_built",
        "measurements": measurements,
        "parameters": copy.deepcopy(sim.parameters),
        "geometry": {
            k: np.asarray(v).tolist() if not isinstance(v, dict) else v
            for k, v in sim.geometry.items()
        },
        "actuators": {
            "left_travel_mm": 1300 * np.pi * 55 / 360 * 0.1,
            "right_travel_mm": 1500 * np.pi * 55 / 360 * 0.1,
            "brake_rate_mm_s": 221.522558906,
            "source": "Existing servo geometry and rate model",
            "uncertainty": None,
        },
        "note": "Null uncertainty means unquantified, never zero. Coefficients below are prior estimates, not all measured.",
    }


def make_aircraft(profile, theta):
    if profile.get("schema") != 1 or profile.get("flight") != "trout_lake":
        raise ValueError("Only the Trout Lake as-built adapter is implemented")
    theta = np.asarray(theta, float)
    if (
        theta.shape != (len(NAMES),)
        or not np.isfinite(theta).all()
        or np.any(theta < BOUNDS[:, 0])
        or np.any(theta > BOUNDS[:, 1])
    ):
        raise ValueError("Fitted parameters outside adapter bounds")
    sim = make_simulator("trout_lake", profile="as_built", substeps=16)
    if set(profile["parameters"]) != set(sim.parameters):
        raise ValueError("Physics parameter schema mismatch")
    sim.parameters = copy.deepcopy(profile["parameters"])
    sim.geometry = copy.deepcopy(profile["geometry"])
    for key in ["inertia", "hinge", "thrust", "imu"]:
        sim.geometry[key] = np.asarray(sim.geometry[key], float)
    for name, (target, key, units, _) in MEASUREMENTS.items():
        m = profile["measurements"][name]
        if (
            m["units"] != units
            or not m.get("source")
            or m.get("fixed") is not True
            or not np.isfinite(m["value"])
        ):
            raise ValueError("Invalid measured input: " + name)
        uncertainty = m["uncertainty"]
        if uncertainty is not None and (
            not np.isfinite(uncertainty) or uncertainty < 0
        ):
            raise ValueError("Invalid uncertainty: " + name)
        (sim.geometry if target == "geometry" else sim.parameters)[key] = m["value"]
    if (
        abs(
            sim.geometry["mass"]
            - sim.geometry["payload_mass"]
            - sim.parameters["canopy_mass"]
        )
        > 1e-6
    ):
        raise ValueError("Masses are inconsistent")
    if (
        min(
            sim.geometry["mass"],
            sim.geometry["payload_mass"],
            sim.parameters["canopy_mass"],
        )
        <= 0
    ):
        raise ValueError("Masses must be positive")
    if abs(sim.parameters["yaw_payload_halfspacing"] - 0.066) > 1e-9:
        raise ValueError(
            "Riser geometry changes require regenerating the routing model; adapter is as-built only"
        )
    inertia = np.array(
        [
            [sim.parameters["Ixx"], sim.parameters["Ixy"], sim.parameters["Ixz"]],
            [sim.parameters["Ixy"], sim.parameters["Iyy"], sim.parameters["Iyz"]],
            [sim.parameters["Ixz"], sim.parameters["Iyz"], sim.parameters["Izz"]],
        ]
    )
    if min(np.linalg.eigvalsh(inertia)) <= 0:
        raise ValueError("Inertia must be positive definite")
    sim.geometry["inertia"] = inertia
    sim.geometry["canopy_mass"] = sim.parameters["canopy_mass"]
    sim.geometry["hinge"] = np.array([sim.parameters["hinge_" + a] for a in "xyz"])
    for key, factor in zip(
        ["rotor_spin_inertia", "joint_k_yaw", "joint_c_yaw", "joint_c_pitch"], theta[:4]
    ):
        sim.parameters[key] *= factor
    original = sim.controls

    def controls(*args):
        out = original(*args)
        out[1:] *= theta[4]
        return out

    sim.controls = controls
    return sim
