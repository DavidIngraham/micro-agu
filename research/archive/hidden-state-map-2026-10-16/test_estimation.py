"""Structural and leakage tests for the conditional estimation experiment."""
import numpy as np
from scipy.spatial.transform import Rotation as R
from run import est, segment


def test_hinge_constraints_and_roundtrip():
    s = segment('trout_lake', 600, 2)
    h = np.r_[s.initial_relative+[.04, -.07, .03], [.2, -.1, .3], [1., -2.]]
    y = s.state(0, h)
    rp, rc = R.from_quat(y[3:7]), R.from_quat(y[16:20])
    ap, ac = rp.apply(s.hinge), rc.apply([0., 0., .85])
    np.testing.assert_allclose(y[:3]+ap, y[13:16]+ac, atol=1e-12)
    np.testing.assert_allclose(y[7:10]+np.cross(y[10:13], ap),
                               y[20:23]+np.cross(y[23:26], ac), atol=1e-12)
    np.testing.assert_allclose(est.hidden_from_state(y)[0], h[:6], atol=1e-12)
    observed = est.observe(y, s.imu)[0]
    angle_error = (R.from_euler('xyz', observed[:3]).inv()*R.from_euler('xyz', s.observation[0, :3])).magnitude()
    assert angle_error < 1e-12
    np.testing.assert_allclose(observed[3:], s.observation[0, 3:], atol=1e-12)


def test_sparse_pattern_contains_every_dependency():
    s = segment('trout_lake', 600, .4)
    graph = est.Graph([s], fit_damping=True)
    x = graph.initial(); r = graph.residual(x); pattern = graph.sparsity().toarray()
    assert pattern.shape == (len(r), len(x))
    for col in range(len(x)):
        bumped = x.copy(); bumped[col] += 1e-6
        delta = abs(graph.residual(bumped)-r)
        assert np.max(delta[pattern[:, col] == 0], initial=0) < 1e-10


def test_forecast_does_not_read_future_navigation_or_motor_measurements():
    s = segment('trout_lake', 600, 2)
    hidden = np.r_[s.initial_relative, np.zeros(3), [1., .5]]
    a = est.predict(s, hidden, [3., 3.], 0, 50)
    s.observation[1:] += 100
    s.rpm[1:] += 1000
    s.voltage[1:] *= .5
    b = est.predict(s, hidden, [3., 3.], 0, 50)
    np.testing.assert_array_equal(a, b)


def test_parameters_are_not_mutated():
    s = segment('trout_lake', 600, .4)
    p = dict(s.parameters)
    graph = est.Graph([s], fit_damping=True)
    graph.residual(graph.initial())
    assert s.parameters == p


def test_divergent_trial_has_infinite_cost_without_changing_factor_shape():
    s = segment('trout_lake', 600, .4)
    graph = est.Graph([s])
    class DivergentKernel:
        def replay_numpy(self, *args):
            raise ValueError('nonfinite or divergent state')
    graph.kernels = lambda _: [DivergentKernel()]
    residual = graph.residual(graph.initial())
    assert len(residual) == graph.sparsity().shape[0]
    assert np.isinf(residual[:15]).all()
    assert np.isfinite(residual[15:]).all()
    assert graph.rejected_edges == 1
