"""Observed motion shape in manual and automatic segments."""

import numpy as np
from scipy.signal import welch, csd, butter, sosfiltfilt
import investigate as x

rows = []
ti, imu = x.read("IMU")
g = np.column_stack([imu[k] for k in ["GyrX", "GyrY", "GyrZ"]]) * 180 / np.pi
for start in [300, 390, 540, 1440, 2100, 2760]:
    t = np.arange(start, start + 30, 0.02)
    r = x.interp(ti, g, t)
    f, p = welch(r, fs=50, nperseg=512, noverlap=256, axis=0)
    band = (f >= 1.2) & (f <= 1.9)
    i = np.flatnonzero(band)[np.argmax(p[band, 2])]
    _, qr = csd(r[:, 2], r[:, 1], fs=50, nperseg=512, noverlap=256)
    bp = sosfiltfilt(
        butter(3, [1.2, 1.9], fs=50, btype="bandpass", output="sos"), r, axis=0
    )[100:-100]
    v, e = np.linalg.eigh(np.cov(bp.T))
    axis = e[:, -1]
    axis *= np.sign(axis[2])
    rows.append(
        dict(
            start=start,
            peak_hz=float(f[i]),
            pitch_per_yaw_amplitude=float(np.sqrt(p[i, 1] / p[i, 2])),
            pitch_relative_to_yaw_phase_deg=float(np.angle(qr[i]) * 180 / np.pi),
            principal_axis=axis.tolist(),
            principal_fraction=float(v[-1] / sum(v)),
        )
    )
x.save("motion-shape.json", rows)
