"""Repeatable, smooth synthetic gust field for viewer demonstrations.

Uniform NED air velocity shared by both bodies; no visual motion injection.
This is an illustrative disturbance, not a calibrated atmospheric spectrum.
"""
import json
import numpy as np

SETTINGS = dict(seed=20261011, rms_ned_m_s=[0.5, 0.5, 0.25],
                frequencies_hz=[0.08, 0.13, 0.21, 0.34, 0.55, 0.89],
                ramp_s=5, mean_wind_ned_m_s=[0, 0, 0],
                model='Uniform smooth random-phase multisine; nominal component RMS, not a Dryden model')


class Gusts:
    def __init__(self):
        self.phase = np.random.default_rng(SETTINGS['seed']).uniform(0, 2*np.pi, (3, 6))
        self.frequency = np.array(SETTINGS['frequencies_hz'])
        weights = np.array([1, 1, .8, .6, .4, .25])
        self.amplitude = np.array(SETTINGS['rms_ned_m_s'])[:, None] * weights * np.sqrt(2 / np.sum(weights**2))

    def at(self, time):
        t = max(0., time)
        ramp = .5 - .5*np.cos(np.pi*min(t/SETTINGS['ramp_s'], 1.))
        return ramp * np.sum(self.amplitude*np.sin(2*np.pi*self.frequency*t + self.phase), axis=1)


def install_backend_wind(backend):
    """Wrap one backend instance's existing wind input; preserve controllers."""
    gusts = Gusts()
    backend._gust_origin = None
    backend._viewer_wind = np.zeros(3)
    step = backend.sim.step_actuated

    def windy_step(*args, **kwargs):
        kwargs['wind'] = backend._viewer_wind
        return step(*args, **kwargs)
    backend.sim.step_actuated = windy_step
    return gusts


def respond_with_wind(backend, packet, original, mean_wind=(0., 0., 0.), gusts_enabled=True):
    # Preserve retransmission behavior and do not create a second gust clock.
    from microagu_sim.compat.json_backend import decode_packet
    rate, frame, _ = decode_packet(packet)
    if frame == backend.frame:
        return backend.last
    if not hasattr(backend, '_gusts'):
        backend._gusts = install_backend_wind(backend)
    if backend.released:
        if backend._gust_origin is None:
            backend._gust_origin = backend.time
        backend._viewer_wind = np.asarray(mean_wind, dtype=float) + (backend._gusts.at(backend.time - backend._gust_origin + .5/rate) if gusts_enabled else np.zeros(3))
    response = original(backend, packet)
    message = json.loads(response)
    # The stock JSON adapter reports calm wind. Keep its sensors consistent with
    # the same air velocity applied to the aerodynamic and propulsion model.
    message['velocity_wind'] = backend._viewer_wind.tolist()
    message['airspeed'] = float(np.linalg.norm(backend.state[7:10] - backend._viewer_wind))
    backend.last = ('\n' + json.dumps(message, allow_nan=False) + '\n').encode()
    return backend.last
