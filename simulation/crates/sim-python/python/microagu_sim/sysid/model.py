"""Six-parameter aircraft adapter for the current as-built paraglider profiles."""

import numpy as np
from scipy.spatial.transform import Rotation

from microagu_sim.compat import dynamics
from microagu_sim.compat.profiles import make_simulator

NAMES = [
    "rotor_inertia_scale",
    "yaw_stiffness_scale",
    "yaw_damping_scale",
    "pitch_damping_scale",
    "effective_pull_scale",
    "actuator_lag_s",
]
BOUNDS = np.array(
    [(0.5, 1.5), (0.5, 1.5), (0.2, 1.5), (0.2, 1.5), (0.85, 1.15), (0, 0.12)]
)
DEFAULT = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.0])


def initialize(sim, segment, alpha, wind):
    if segment.initial_state is not None:
        return segment.initial_state.copy()
    obs = segment.observation[0]
    rp = Rotation.from_euler("xyz", obs[:3])
    wp = rp.apply(obs[3:6])
    g = sim.geometry
    v = obs[6:9] - np.cross(wp, rp.apply(g["imu"]))
    e = v - wind
    if np.linalg.norm(e) < 0.1:
        raise ValueError("Insufficient airflow for canopy initialization")
    e = e / np.linalg.norm(e)
    axial = rp.inv().apply(v - wind)[0]
    thrust = dynamics.lib.mapped_thrust(segment.rpm, float(axial))
    force = g["mass"] * segment.force_initial - rp.apply([thrust, 0, 0])
    down = -(force - e * (force @ e))
    if np.linalg.norm(down) < 1e-8:
        raise ValueError("Degenerate canopy initialization force")
    down /= np.linalg.norm(down)
    span = np.cross(down, e)
    span /= np.linalg.norm(span)
    rc = Rotation.from_matrix(
        np.column_stack(
            [
                np.cos(alpha) * e - np.sin(alpha) * down,
                span,
                np.sin(alpha) * e + np.cos(alpha) * down,
            ]
        )
    )
    pr, cr = rp.apply(g["hinge"]), rc.apply([0, 0, 0.85])
    return np.r_[
        np.zeros(3),
        rp.as_quat(),
        v,
        wp,
        pr - cr,
        rc.as_quat(),
        v + np.cross(wp, pr) - np.cross(wp, cr),
        wp,
    ]


def replay(segment, theta=DEFAULT, nuisance=None, substeps=32, alpha=0.35):
    theta = np.asarray(theta, float)
    if (
        theta.shape != (6,)
        or not np.isfinite(theta).all()
        or np.any(theta < BOUNDS[:, 0])
        or np.any(theta > BOUNDS[:, 1])
    ):
        raise ValueError("Parameters outside model bounds")
    nuisance = np.zeros(3) if nuisance is None else np.asarray(nuisance, float)
    if nuisance.shape != (3,) or not np.isfinite(nuisance).all():
        raise ValueError("Expected canopy-angle, north-wind, east-wind corrections")
    sim = make_simulator(segment.flight, profile="as_built", substeps=substeps)
    for key, value in zip(
        ["rotor_spin_inertia", "joint_k_yaw", "joint_c_yaw", "joint_c_pitch"], theta[:4]
    ):
        sim.parameters[key] *= value
    wind = segment.wind + np.r_[nuisance[1:], 0.0]
    y0 = initialize(sim, segment, alpha + nuisance[0], wind)
    u = np.array([sim.controls(*row) for row in segment.commands])
    u[:, 1:] *= theta[4]
    if theta[5] > 1e-6:
        decay = np.exp(-0.04 / theta[5])
        applied = u[0, 1:].copy()
        for j in range(1, len(u)):
            applied = decay * applied + (1 - decay) * u[j, 1:]
            u[j, 1:] = applied
    y, rpm = dynamics.causal_replay(
        y0,
        u,
        sim.parameters,
        sim.geometry["mass"],
        wind,
        segment.voltage,
        segment.rpm,
        dt=0.04,
        substeps=substeps,
    )
    if not np.isfinite(y).all():
        raise ValueError("Nonfinite replay")
    rates = Rotation.from_quat(y[:, 3:7]).inv().apply(y[:, 10:13]) * 180 / np.pi
    return rates, y, rpm
