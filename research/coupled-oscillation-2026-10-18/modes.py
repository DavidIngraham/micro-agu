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
seed = np.load(Path(compat.__file__).parent / "equilibrium-trim-trout_lake-0.0-0.0.npz")
print("seed", seed.files, flush=True)


def solve(sim, base, unused):
    # Both lines engaged: fixed 6 mm common effective pull avoids the slack kink.
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
                x[-2],
                6 + sim.parameters["left_offset"] * 1000 - x[-1] / 2,
                6 + sim.parameters["right_offset"] * 1000 + x[-1] / 2,
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
            y[9],
            Rotation.from_quat(y[3:7]).as_euler("xyz")[2],
        ]

    x0 = np.r_[np.zeros(15), 0.45, 0.0]
    lo = np.r_[np.full(6, -1.0), np.full(3, -3.0), np.full(6, -3.0), 0.0, -8.0]
    hi = np.r_[np.full(6, 1.0), np.full(3, 3.0), np.full(6, 3.0), 0.65, 8.0]
    fit = least_squares(
        residual,
        x0,
        bounds=(lo, hi),
        x_scale=np.r_[np.ones(15), 0.1, 5.0],
        max_nfev=200,
        ftol=1e-11,
        xtol=1e-11,
        gtol=1e-10,
    )
    y, u, rpm = unpack(fit.x)
    return dict(
        state=y, command=u, rpm=rpm, success=bool(fit.success), residual=residual(fit.x)
    )


rows = []
for label in [
    "baseline",
    "candidate",
    "no_rotor_gyro",
    "diagonal_payload_inertia",
    "half_yaw_stiffness",
    "double_yaw_stiffness",
    "half_pitch_damping",
    "half_yaw_damping",
    "half_rotor_gyro",
    "double_rotor_gyro",
]:
    sim = make_simulator("trout_lake", profile="as_built", substeps=32)
    if label == "candidate":
        sim.parameters.update(
            json.loads(
                (
                    OUT.parent
                    / "trout-lake-fidelity-2026-10-18/candidate-parameters.json"
                ).read_text()
            )["parameters"]
        )
    if label == "no_rotor_gyro":
        sim.parameters["rotor_spin_inertia"] = 0.0
    if label == "diagonal_payload_inertia":
        for key in ["Ixy", "Ixz", "Iyz"]:
            sim.parameters[key] = 0.0
    if label == "half_yaw_stiffness":
        sim.parameters["joint_k_yaw"] *= 0.5
    if label == "double_yaw_stiffness":
        sim.parameters["joint_k_yaw"] *= 2
    if label == "half_pitch_damping":
        sim.parameters["joint_c_pitch"] *= 0.5
    if label == "half_yaw_damping":
        sim.parameters["joint_c_yaw"] *= 0.5
    if label == "half_rotor_gyro":
        sim.parameters["rotor_spin_inertia"] *= 0.5
    if label == "double_rotor_gyro":
        sim.parameters["rotor_spin_inertia"] *= 2
    fit = solve(
        sim,
        seed["state"],
        [0.45, 10.0, 0.0] if label == "candidate" else [0.45, 0.0, 0.0],
    )
    y, u, rpm = fit["state"], fit["command"], fit["rpm"]
    h = float(os.environ.get("MICROAGU_MODE_DT", ".004"))
    eps = 1e-5
    if max(abs(fit["residual"])) > 1e-4:
        rows.append(
            dict(
                variant=label,
                trim_success=False,
                trim_max_residual=float(max(abs(fit["residual"]))),
                reason="Solver termination is not a converged equilibrium; modal/response claims withheld.",
            )
        )
        continue
    base, _ = sim.step(y, rpm, u, dt=h)
    cols = []
    for k in range(15):
        x = np.eye(15)[k] * eps
        plus, _ = sim.step(perturb(y, x, sim.geometry), rpm, u, dt=h)
        minus, _ = sim.step(perturb(y, -x, sim.geometry), rpm, u, dt=h)
        cols.append((difference(base, plus) - difference(base, minus)) / (2 * eps))
    jac = np.column_stack(cols)
    up = u.copy()
    um = u.copy()
    up[2] += 0.005
    up[1] -= 0.005
    um[2] -= 0.005
    um[1] += 0.005
    yp, _ = sim.step(y, rpm, up, dt=h)
    ym, _ = sim.step(y, rpm, um, dt=h)
    B = (difference(base, yp) - difference(base, ym)) / 0.02
    response = np.linalg.solve(np.exp(2j * np.pi * 1.5625 * h) * np.eye(15) - jac, B)[
        9:12
    ]
    forced = dict(
        frequency_hz=1.5625,
        pitch_per_yaw_amplitude=float(abs(response[1] / response[2])),
        pitch_relative_to_yaw_phase_deg=float(
            np.angle(response[1] / response[2]) * 180 / np.pi
        ),
        yaw_gain_deg_s_per_mm=float(abs(response[2]) * 180 / np.pi),
    )
    vals, vec = np.linalg.eig(jac)
    lam = np.log(vals.astype(complex)) / h
    modes = []
    for j, v in enumerate(lam):
        if v.imag <= 0.1:
            continue
        z = vec[9:12, j]
        scale = np.linalg.norm(z)
        modes.append(
            dict(
                frequency_hz=float(v.imag / (2 * np.pi)),
                decay_per_s=float(-v.real),
                damping_ratio=float(-v.real / abs(v)),
                payload_rate_magnitude=(abs(z) / max(scale, 1e-12)).tolist(),
                pitch_yaw_phase_deg=float(np.angle(z[1] * np.conj(z[2])) * 180 / np.pi),
            )
        )
    rows.append(
        dict(
            variant=label,
            trim_success=fit["success"],
            trim_max_residual=float(max(abs(fit["residual"]))),
            command=u.tolist(),
            rpm=float(rpm),
            forced_response=forced,
            modes=modes,
        )
    )
    print(label, rows[-1], flush=True)
(OUT / ("model-modes.json" if h == 0.004 else f"model-modes-{h}.json")).write_text(
    json.dumps(rows, indent=2) + "\n"
)
