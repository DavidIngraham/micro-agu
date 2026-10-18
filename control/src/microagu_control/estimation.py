"""Conditional batch MAP estimation of hidden canopy motion and horizontal wind.

Requires the ``analysis`` extra. Payload navigation is conditioned on, rather
than counted again alongside its source IMU/GNSS. Soft shooting defects include
measurement and model discrepancy; their scales are assumptions, not sensor
covariances. This is an offline research estimator, not an AHRS replacement.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation as R

from microagu_sim.compat import dynamics


@dataclass
class Segment:
    name: str
    time: np.ndarray
    observation: np.ndarray  # Euler xyz, body rates, IMU-point NED velocity
    commands: np.ndarray  # already converted to kernel units
    rpm: np.ndarray
    voltage: np.ndarray
    parameters: dict
    mass: float
    imu: np.ndarray
    hinge: np.ndarray
    initial_relative: np.ndarray
    wind_guess: np.ndarray
    stride: int = 10  # 0.4 s at 25 Hz

    def __post_init__(self):
        n = len(self.time)
        if not isinstance(self.stride, int) or self.stride < 1 or n < 2 or (n-1) % self.stride:
            raise ValueError('Segment must contain a whole number of knot intervals')
        if not np.allclose(np.diff(self.time), .04, atol=1e-5):
            raise ValueError('Expected 25 Hz observations and commands')
        if self.observation.shape != (n, 9) or self.commands.shape != (n, 3):
            raise ValueError('Invalid observation or command shape')
        if self.rpm.shape != (n,) or self.voltage.shape != (n,):
            raise ValueError('Motor observations must match the time array')
        if any(np.asarray(a).shape != (3,) for a in [self.imu, self.hinge, self.initial_relative]):
            raise ValueError('Geometry and initial rotation must be three-vectors')
        if np.asarray(self.wind_guess).shape != (2,):
            raise ValueError('Wind guess must contain north and east components')
        if not all(np.isfinite(a).all() for a in
                   [self.time, self.observation, self.commands, self.rpm, self.voltage,
                    self.imu, self.hinge, self.initial_relative, self.wind_guess]):
            raise ValueError('Nonfinite input')

    @property
    def knots(self):
        return np.arange(0, len(self.time), self.stride)

    def state(self, index, hidden):
        """Enforce hinge position/velocity constraints; translation gauge is zero."""
        obs = self.observation[index]
        rp = R.from_euler('xyz', obs[:3])
        rc = rp * R.from_rotvec(hidden[:3])
        wp = rp.apply(obs[3:6])
        wc = wp + rp.apply(hidden[3:6])
        vp = obs[6:9] - np.cross(wp, rp.apply(self.imu))
        ap, ac = rp.apply(self.hinge), rc.apply([0., 0., .85])
        return np.r_[np.zeros(3), rp.as_quat(), vp, wp,
                     ap-ac, rc.as_quat(), vp+np.cross(wp, ap)-np.cross(wc, ac), wc]


def observe(state, imu):
    state = np.atleast_2d(state)
    rp = R.from_quat(state[:, 3:7])
    return np.column_stack([rp.as_euler('xyz'), rp.inv().apply(state[:, 10:13]),
                            state[:, 7:10]+np.cross(state[:, 10:13], rp.apply(imu))])


def hidden_from_state(state):
    state = np.atleast_2d(state)
    rp, rc = R.from_quat(state[:, 3:7]), R.from_quat(state[:, 16:20])
    return np.column_stack([(rp.inv()*rc).as_rotvec(),
                            rp.inv().apply(state[:, 23:26]-state[:, 10:13])])


class Graph:
    """Sparse nonlinear multiple-shooting graph with shared damping parameters.

    Each knot has relative rotation-vector (3), relative world angular velocity
    expressed in payload axes (3), and horizontal NED wind (2). Every edge runs
    the actual Rust simulator and compares payload measurements and canopy
    continuity. Wind has a random-walk prior, independent of the old smoother.
    """
    def __init__(self, segments, damping=(3., 3.), fit_damping=False,
                 wind_diffusion=.10, discrepancy=1., substeps=32):
        self.segments = segments
        self.damping = np.asarray(damping, dtype=float)
        self.fit_damping = fit_damping
        self.wind_diffusion = wind_diffusion
        self.discrepancy = discrepancy
        self.substeps = substeps
        if (self.damping.shape != (2,) or not np.isfinite(self.damping).all()
                or np.any(self.damping <= 0) or wind_diffusion <= 0 or discrepancy <= 0):
            raise ValueError('Positive damping and discrepancy scales required')
        self.slices = []
        count = 0
        for segment in segments:
            size = len(segment.knots)*8
            self.slices.append(slice(count, count+size))
            count += size
        self.nuisance_size = count
        self.size = count + (2 if fit_damping else 0)
        self._kernel_key = None
        self._kernels = None
        self.rejected_edges = 0

    def kernels(self, damping):
        key = tuple(damping)
        if key != self._kernel_key:
            self._kernels = []
            for s in self.segments:
                p = dict(s.parameters)
                p['joint_c_pitch'] *= damping[0]
                p['joint_c_yaw'] *= damping[1]
                self._kernels.append(dynamics._kernel(p, s.mass))
            self._kernel_key = key
        return self._kernels

    def initial(self, angle_offset=0.):
        x = np.zeros(self.size)
        for s, sl in zip(self.segments, self.slices):
            z = x[sl].reshape(-1, 8)
            z[:, :3] = s.initial_relative
            z[:, 1] += angle_offset
            z[:, 6:8] = s.wind_guess
        if self.fit_damping:
            x[-2:] = np.log(self.damping)
        return x

    def residual(self, x):
        damping = np.exp(x[-2:]) if self.fit_damping else self.damping
        output = []
        for s, sl, kernel in zip(self.segments, self.slices, self.kernels(damping)):
            z = x[sl].reshape(-1, 8)
            for j, (a, b) in enumerate(zip(s.knots[:-1], s.knots[1:])):
                dt = s.time[b]-s.time[a]
                try:
                    pred = kernel.replay_numpy(
                        np.ascontiguousarray(s.state(a, z[j])),
                        np.ascontiguousarray(s.commands[a:b+1].ravel()),
                        np.r_[z[j, 6:8], 0.], float(s.voltage[a]), float(s.rpm[a]),
                        .04, self.substeps, True)[-1, :26]
                except ValueError as error:
                    if 'nonfinite' not in str(error) and 'divergent' not in str(error):
                        raise
                    # A trial optimizer step can leave the stable integration
                    # domain. Infinite cost makes TRF reject it and shrink its
                    # trust region, rather than fabricate a finite trajectory.
                    self.rejected_edges += 1
                    output.extend([np.inf]*15)
                    output.extend((z[j+1, 6:8]-z[j, 6:8])/(self.wind_diffusion*np.sqrt(dt)))
                    continue
                target = s.state(b, z[j+1])
                pr, tr = R.from_quat(pred[3:7]), R.from_quat(target[3:7])
                pc, tc = R.from_quat(pred[16:20]), R.from_quat(target[16:20])
                measured = observe(pred, s.imu)[0]
                # Effective discrepancy scales per shooting interval; not raw
                # sensor noise. Adjacent residual correlations are not modeled.
                output.extend((tr.inv()*pr).as_rotvec()/(.08*self.discrepancy))
                output.extend((measured[3:6]-s.observation[b, 3:6])/(.20*self.discrepancy))
                output.extend((measured[6:9]-s.observation[b, 6:9])/(.35*self.discrepancy))
                output.extend((tc.inv()*pc).as_rotvec()/(.08*np.sqrt(dt)*self.discrepancy))
                output.extend(tr.inv().apply(pred[23:26]-target[23:26])/(.30*np.sqrt(dt)*self.discrepancy))
                output.extend((z[j+1, 6:8]-z[j, 6:8])/(self.wind_diffusion*np.sqrt(dt)))
            # Weak initial-state and absolute-wind priors. No EKF/smoothed wind
            # observations, no force observations derived from the same nav data.
            output.extend((z[0, :3]-s.initial_relative)/.5)
            output.extend(z[0, 3:6]/1.)
            output.extend(z[0, 6:8]/5.)
        if self.fit_damping:
            output.extend((x[-2:]-np.log(3.))/1.5)
        return np.asarray(output)

    def sparsity(self):
        rows = sum((len(s.knots)-1)*17+8 for s in self.segments)+(2 if self.fit_damping else 0)
        pattern = lil_matrix((rows, self.size), dtype=int)
        row = 0
        for s, sl in zip(self.segments, self.slices):
            for j in range(len(s.knots)-1):
                pattern[row:row+17, sl.start+j*8:sl.start+(j+2)*8] = 1
                if self.fit_damping:
                    pattern[row:row+15, -2:] = 1
                row += 17
            pattern[row:row+8, sl.start:sl.start+8] = 1
            row += 8
        if self.fit_damping:
            pattern[row:row+2, -2:] = 1
        return pattern.tocsr()

    def solve(self, initial=None, max_nfev=80, tolerance=2e-4):
        lower = np.tile([-1.4]*3+[-5.]*3+[-15.]*2, self.nuisance_size//8)
        upper = -lower
        if self.fit_damping:
            lower, upper = np.r_[lower, np.log([.5, .5])], np.r_[upper, np.log([12., 12.])]
        result = least_squares(self.residual, self.initial() if initial is None else initial,
                               bounds=(lower, upper), jac_sparsity=self.sparsity(),
                               loss='soft_l1', f_scale=1., x_scale='jac',
                               max_nfev=max_nfev, ftol=tolerance, xtol=tolerance, gtol=tolerance,
                               tr_options={'atol': 1e-5, 'btol': 1e-5})
        return result

    def damping_values(self, x):
        return np.exp(x[-2:]) if self.fit_damping else self.damping


def predict(segment, hidden, damping, start, stop, substeps=32):
    """Open-loop forecast: no observation/RPM/wind updates after start."""
    p = dict(segment.parameters)
    p['joint_c_pitch'] *= damping[0]
    p['joint_c_yaw'] *= damping[1]
    kernel = dynamics._kernel(p, segment.mass)
    return kernel.replay_numpy(np.ascontiguousarray(segment.state(start, hidden)),
                               np.ascontiguousarray(segment.commands[start:stop+1].ravel()),
                               np.r_[hidden[6:8], 0.], float(segment.voltage[start]),
                               float(segment.rpm[start]), .04, substeps, True)
