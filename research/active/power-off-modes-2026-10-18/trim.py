"""Model-only trim/eigenmode sensitivity; not a fit to flight data."""

import json
import os
from pathlib import Path
import numpy as np
from microagu_sim import compat
from microagu_sim.compat.profiles import make_simulator
from microagu_sim.compat.equilibrium import perturb, difference
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent




def solve(sim, base, throttle, common=12):
    # Both lines engaged: fixed 12 mm common effective pull avoids the slack kink.
    base = base.copy()
    yaw = Rotation.from_quat(base[3:7]).as_euler("xyz")[2]
    rz = Rotation.from_euler("z", -yaw)
    for sl in [slice(3, 7), slice(16, 20)]:
        base[sl] = (rz * Rotation.from_quat(base[sl])).as_quat()
    for sl in [slice(7, 10), slice(10, 13), slice(20, 23), slice(23, 26)]:
        base[sl] = rz.apply(base[sl])

    def unpack(x):
        y = perturb(base, x[:15], sim.geometry)
        u = np.array(
            [
                throttle,
                common + sim.parameters["left_offset"] * 1000 - x[-1] / 2,
                common + sim.parameters["right_offset"] * 1000 + x[-1] / 2,
            ]
        )
        rpm = (
            743.5778222336925
            * 22.2
            * max(u[0] - 0.0005141016498671315, 0) ** 0.9086807307118114
        )
        return y, u, rpm

    def residual(x):
        y, u, rpm = unpack(x)
        n, _ = sim.step(y, rpm, u, dt=0.004)
        return np.r_[
            difference(y, n) / 0.004,
            Rotation.from_quat(y[3:7]).as_euler("xyz")[2],
        ]

    x0 = np.r_[np.zeros(15), 0.0]
    lo = np.r_[np.full(6, -1.0), np.full(3, -3.0), np.full(6, -3.0), -20.0]
    hi = np.r_[np.full(6, 1.0), np.full(3, 3.0), np.full(6, 3.0), 20.0]
    fit = least_squares(
        residual,
        x0,
        bounds=(lo, hi),
        x_scale=np.r_[np.ones(15), 5.0],
        max_nfev=200,
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-10,
    )
    y, u, rpm = unpack(fit.x)
    return dict(
        state=y, command=u, rpm=rpm, success=bool(fit.success), residual=residual(fit.x)
    )


