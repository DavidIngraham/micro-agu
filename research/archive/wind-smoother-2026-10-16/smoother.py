"""Robust horizontal-wind Kalman/RTS smoother, with navigation treated as input.

Conditional covariance excludes systematic navigation/sideslip errors; callers
must evaluate those separately. No aerodynamic parameters or EKF wind inputs.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def smooth(t, velocity, euler, *, diffusion=.05, beta_deg=0., yaw_deg=0., wind_down=0., bias_sigma=.5, iterations=6):
    t, velocity, euler = map(np.asarray, (t, velocity, euler))
    assert len(t) > 2 and np.all(np.diff(t) > 0)
    attitude = euler.copy()
    attitude[:, 2] += np.deg2rad(yaw_deg)
    rotation = Rotation.from_euler('xyz', attitude).as_matrix()
    axis = rotation[:, :, 1] - np.tan(np.deg2rad(beta_deg))*rotation[:, :, 0]
    heading_axis = axis[:, :2]
    # One constant nuisance offset per segment represents persistent sideslip or
    # heading error. It is not a freely varying aerodynamic state. Without this
    # correlated allowance, small heading changes can create false confidence.
    h = np.column_stack((heading_axis, np.ones(len(t))))
    z = np.sum(axis*velocity, axis=1)-axis[:, 2]*wind_down
    n = len(t)
    estimate = np.zeros((n, 3))
    weights = np.ones(n)
    for _ in range(iterations):
        air = velocity - np.column_stack((estimate[:, :2], np.full(n, wind_down)))
        speed = np.clip(np.linalg.norm(air, axis=1), 1., 20.)
        # 0.5 m/s velocity floor and 5 degrees combined angular allowance.
        variance = .5**2 + (speed*np.deg2rad(5))**2
        residual = z-np.sum(h*estimate, axis=1)
        weights = np.minimum(1., 1.5*np.sqrt(variance)/np.maximum(abs(residual), 1e-9))
        xf, pf, xp, pp = np.zeros((n, 3)), np.zeros((n, 3, 3)), np.zeros((n, 3)), np.zeros((n, 3, 3))
        x, p = np.zeros(3), np.diag([25., 25., max(bias_sigma, .001)**2])
        for i in range(n):
            if i:
                p = p + np.diag([diffusion**2, diffusion**2, 0.])*(t[i]-t[i-1])
            xp[i], pp[i] = x, p
            r = variance[i]/weights[i]
            gain = p@h[i]/(h[i]@p@h[i]+r)
            x = x+gain*(z[i]-h[i]@x)
            a = np.eye(3)-np.outer(gain, h[i])
            p = a@p@a.T + np.outer(gain, gain)*r
            xf[i], pf[i] = x, p
        estimate, covariance = xf.copy(), pf.copy()
        for i in range(n-2, -1, -1):
            g = np.linalg.solve(pp[i+1], pf[i].T).T
            estimate[i] = xf[i]+g@(estimate[i+1]-xp[i+1])
            covariance[i] = pf[i]+g@(covariance[i+1]-pp[i+1])@g.T
        covariance = .5*(covariance+covariance.transpose(0, 2, 1))
    # Local geometry only: excludes the temporal prior and absolute noise scale.
    observability = np.zeros(n)
    for i in range(n):
        selected = abs(t-t[i]) <= 30
        eig = np.linalg.eigvalsh(heading_axis[selected].T@heading_axis[selected])
        observability[i] = eig[0]/max(eig[-1], 1e-12)
    return dict(wind=estimate[:, :2], covariance=covariance[:, :2, :2], lateral_bias=estimate[:, 2], weights=weights,
                residual=z-np.sum(h*estimate, axis=1), observability=observability)
