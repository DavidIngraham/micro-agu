"""Gain synthesis and observer-in-loop robustness screen. Not EKF/SITL qualification."""

import numpy as np
from scipy.linalg import solve_discrete_are

from ..sysid.aircraft import make_aircraft
from .linearize import DT, US, XS, Chart, command, jacobian, trim

MEASURED = [0, 1, 5, 6, 7, 8, 9, 10, 15]


def output_matrix(z):
    h = np.zeros((3, 16))
    h[0, 15] = 1
    roll, pitch, _ = z["EULER"]
    h[1, 9:11] = (
        np.array([np.sin(roll), np.cos(roll)])
        / np.cos(pitch)
        * z["XS"][9:11]
        / np.deg2rad(10)
    )
    h[2, 5:8] = z["VELOCITY"] / np.linalg.norm(z["VELOCITY"]) * z["XS"][5:8] / 0.5
    return h


def linearize(profile, theta, common):
    sim = make_aircraft(profile, theta)
    state, u, rpm, error = trim(sim, common=common)
    chart = Chart(sim, state, rpm)

    def advance(x, control):
        y, rotor = chart.decode(x)
        y, rotor = sim.step(y, rotor, command(control), dt=DT)
        return chart.encode(y, rotor)

    zero = np.zeros(16)
    a = jacobian(lambda x: advance(x, u), zero) * XS[None, :] / XS[:, None]
    b = jacobian(lambda c: advance(zero, c), u) * US[None, :] / XS[:, None]
    z = {
        "A": a,
        "B": b,
        "XS": XS,
        "US": US,
        "EULER": chart.euler,
        "VELOCITY": chart.velocity,
    }
    return z | {
        "H": output_matrix(z),
        "theta": np.asarray(theta),
        "state": state,
        "rpm": rpm,
        "command": u,
        "trim_residual": error,
        "zero_drift": float(np.linalg.norm(advance(zero, u))),
    }


def synthesize(plant, reference, weights):
    a = np.eye(19)
    a[:16, :16] = plant["A"]
    a[16:, :16] = DT * plant["H"]
    b = np.zeros((19, 3))
    b[:16] = plant["B"]
    q = np.diag(
        np.r_[np.full(16, 0.02), weights["integral"], 3, weights["speed_integral"]]
    )
    q[:16, :16] += (
        plant["H"].T @ np.diag([weights["height"], 1, weights["speed"]]) @ plant["H"]
    )
    q[9, 9] += weights["pitch"]
    q[12, 12] += weights["pitch"]
    r = np.diag([weights["throttle"], weights["common"], 1000])
    if (
        not np.isfinite(q).all()
        or not np.isfinite(r).all()
        or min(weights.values()) < 0
        or min(np.diag(r)) <= 0
    ):
        raise ValueError("Invalid synthesis weights")
    p = solve_discrete_are(a, b, q, r)
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    # Fixed firmware coordinate/observer contract; feedback only, same measured sensors.
    result = {key: value.copy() for key, value in reference.items()}
    result["K"] = k
    result["AW"] = np.linalg.inv(reference["US"][:, None] * k[:, 16:])
    return result


def closed_loop(plant, controller):
    """Exact Jacobian of stated local sampled loop, including hidden brake lag.

    State: plant(16), previous corrected observer(16), integrals(3),
    previous allocated demand(3), brake lag(2). No canopy truth is measured.
    Integrals use current measurement, as in AP; prediction uses previous demand.
    Lag follows the sysid update-before-physics convention. Rate limits and
    unilateral common brake require subsequent nonlinear qualification.
    """
    c = np.eye(16)[MEASURED]
    h = plant["H"]
    decay = np.exp(-DT / max(float(plant["theta"][5]), 1e-12))

    def advance(v):
        x, previous, integral, last, brake = (
            v[:16],
            v[16:32],
            v[32:35],
            v[35:38],
            v[38:],
        )
        prediction = controller["A"] @ previous + controller["B"] @ last
        estimate = prediction + controller["L"] @ (c @ x - c @ prediction)
        integral = integral + DT * h @ x
        demand = (
            -controller["K"][:, :16] @ estimate - controller["K"][:, 16:] @ integral
        )
        brake = decay * brake + (1 - decay) * demand[1:]
        applied = np.r_[demand[0], brake]
        x = plant["A"] @ x + plant["B"] @ applied
        return np.r_[x, estimate, integral, demand, brake]

    return np.column_stack([advance(e) for e in np.eye(40)])


def screen(plant, controller):
    loop = closed_loop(plant, controller)
    poles = np.linalg.eigvals(loop)
    active = poles[np.abs(poles) > 1e-10]
    continuous = np.log(active.astype(complex)) / DT
    oscillatory = continuous[np.abs(continuous.imag) > 1e-3]
    damping = -oscillatory.real / np.abs(oscillatory)
    return {
        "spectral_radius": float(max(abs(poles))),
        "stable": bool(max(abs(poles)) < 1 - 1e-8),
        "minimum_oscillatory_damping": float(min(damping)) if len(damping) else None,
        "poles": [[float(z.real), float(z.imag)] for z in poles],
    }
