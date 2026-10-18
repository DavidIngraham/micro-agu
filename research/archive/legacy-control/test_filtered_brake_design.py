"""Check augmentation against the unaugmented Riccati solution and filter physics."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.linalg import solve_discrete_are


def test_memory_cost_and_zero_weight_equivalence(tmp_path):
    path = Path(__file__).resolve().parents[1] / 'tools/design_mimo.py'
    spec = importlib.util.spec_from_file_location('design_mimo', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for kwargs in [dict(brake_memory_weight=float('nan')), dict(brake_memory_tau=0),
                   dict(brake_memory_weight=-1), dict(brake_memory_tau=float('inf'))]:
        with pytest.raises(ValueError):
            module.design(tmp_path, **kwargs)
    for weight in (0, 3000):
        out = tmp_path / str(weight)
        module.design(out, brake_memory_weight=weight, brake_memory_tau=3)
        model = np.load(out / 'design.npz')
        a, b, q, r, k = (model[key] for key in ('AA', 'BB', 'Q', 'R', 'K'))
        assert max(abs(np.linalg.eigvals(a-b@k))) < 1
        # A unit common-brake step asymptotes to one, with the specified time constant.
        z = 0.0
        for _ in range(150):
            z = a[19, 19]*z+b[19, 1]
        np.testing.assert_allclose(z, 1-np.exp(-1), atol=1e-12)
        if weight == 0:
            p = solve_discrete_are(a[:19, :19], b[:19], q[:19, :19], r)
            old_k = np.linalg.solve(r+b[:19].T@p@b[:19], b[:19].T@p@a[:19, :19])
            np.testing.assert_allclose(k[:, :19], old_k, atol=1e-8)
            np.testing.assert_allclose(k[:, 19], 0, atol=1e-12)
        else:
            assert k[1, 19] > 0
