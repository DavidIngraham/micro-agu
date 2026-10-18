"""Local Rust-physics linearization. Promoted from tools/design_mimo.py (AI-assisted)."""

from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from microagu_sim import compat
from microagu_sim.compat.equilibrium import difference, perturb

DT = 0.02
STROKES = np.array([1300, 1500]) * np.pi * 55 / 360 * 0.1
XS = np.r_[
    np.full(5, np.deg2rad(10)), np.full(3, 0.5), np.full(6, np.deg2rad(30)), 1.0, 1.0
]
US = np.array([0.1, 0.1, 0.1])


def command(u):
    throttle, common, differential = u
    return np.r_[
        throttle, STROKES * (common + np.maximum([-differential, differential], 0))
    ]


class Chart:
    """Heading-invariant 16-state chart, with body-frame velocities and rates."""

    def __init__(self, sim, state, rpm):
        self.sim, self.base, self.rpm = sim, state.copy(), rpm
        self.base[13:16] -= self.base[:3]
        self.base[:3] = 0
        payload = Rotation.from_quat(state[3:7])
        canopy = Rotation.from_quat(state[16:20])
        self.euler = payload.as_euler("xyz")
        self.relative = payload.inv() * canopy
        self.velocity = payload.inv().apply(state[7:10])
        self.payload_rates = payload.inv().apply(state[10:13])
        self.canopy_rates = canopy.inv().apply(state[23:26])

    def decode(self, x):
        payload = Rotation.from_euler("xyz", self.euler + np.r_[x[:2], 0])
        canopy = payload * self.relative * Rotation.from_rotvec(x[2:5])
        state = self.base.copy()
        state[2] = -x[15]
        state[3:7], state[16:20] = payload.as_quat(), canopy.as_quat()
        state[7:10] = payload.apply(self.velocity + x[5:8])
        state[10:13] = payload.apply(self.payload_rates + x[8:11])
        state[23:26] = canopy.apply(self.canopy_rates + x[11:14])
        arm_payload = payload.apply(self.sim.geometry["hinge"])
        arm_canopy = canopy.apply([0, 0, 0.85])
        state[13:16] = state[:3] + arm_payload - arm_canopy
        state[20:23] = (
            state[7:10]
            + np.cross(state[10:13], arm_payload)
            - np.cross(state[23:26], arm_canopy)
        )
        return state, self.rpm + 1000 * x[14]

    def encode(self, state, rpm):
        payload = Rotation.from_quat(state[3:7])
        canopy = Rotation.from_quat(state[16:20])
        return np.r_[
            payload.as_euler("xyz")[:2] - self.euler[:2],
            (self.relative.inv() * payload.inv() * canopy).as_rotvec(),
            payload.inv().apply(state[7:10]) - self.velocity,
            payload.inv().apply(state[10:13]) - self.payload_rates,
            canopy.inv().apply(state[23:26]) - self.canopy_rates,
            (rpm - self.rpm) / 1000,
            -state[2],
        ]


def trim(sim, voltage=22.2, common=0.0):
    seed = np.load(
        Path(compat.__file__).parent / f"equilibrium-trim-{sim.flight}-0.0-0.0.npz"
    )
    base = seed["state"].copy()

    def unpack(x):
        state = perturb(base, x[:15], sim.geometry)
        u = np.array([x[15], common, x[16]])
        rpm = (
            743.5778222336925
            * voltage
            * max(u[0] - 0.0005141016498671315, 0) ** 0.9086807307118114
        )
        return state, u, rpm

    def residual(x):
        state, u, rpm = unpack(x)
        next_state, _ = sim.step(state, rpm, command(u), voltage=voltage, dt=0.004)
        return np.r_[
            difference(state, next_state) / 0.004,
            state[9],
            Rotation.from_quat(state[3:7]).as_euler("xyz")[2],
        ]

    # The unilateral brake map has distinct left/right branches. A seed on
    # the wrong branch can stop at the dead zone without finding equilibrium.
    # Preserve the established seed first, then try the opposite branch.
    for differential in (0.02, -0.1):
        fit = least_squares(
            residual,
            np.r_[np.zeros(15), 0.4, differential],
            bounds=(np.r_[np.full(15, -3), 0.01, -0.3], np.r_[np.full(15, 3), 0.8, 0.3]),
            max_nfev=300,
            ftol=1e-11,
            xtol=1e-11,
            gtol=1e-10,
        )
        error = float(np.max(np.abs(residual(fit.x))))
        if fit.success and error <= 1e-6:
            break
    else:
        raise RuntimeError(f"trim failed: {fit.message}, residual {error}")
    return (*unpack(fit.x), error)


def jacobian(function, value, epsilon=1e-5):
    return np.column_stack(
        [
            (function(value + delta) - function(value - delta)) / (2 * epsilon)
            for delta in np.eye(len(value)) * epsilon
        ]
    )
