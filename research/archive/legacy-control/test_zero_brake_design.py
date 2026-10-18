"""The boundary trim must be physical and its synthesis must not request negative travel."""
import importlib.util
import json
from pathlib import Path

import numpy as np


def test_zero_brake_synthesis(tmp_path):
    path = Path(__file__).resolve().parents[1] / 'tools/design_mimo.py'
    spec = importlib.util.spec_from_file_location('design_mimo', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.command

    def physical_command(u):
        assert u[1] >= 0, 'Boundary linearization evaluated negative common brake'
        return original(u)

    module.command = physical_command
    module.design(tmp_path, common=0, common_effort=1000, pitch_weight=30)
    report = json.loads((tmp_path / 'design.json').read_text())
    assert report['trim_command'][1] == 0
    assert report['trim_residual'] < 1e-8
    assert report['zero_drift'] < 1e-8
    assert report['common_derivative'] == 'right-sided'
    model = np.load(tmp_path / 'design.npz')
    assert np.linalg.norm(model['B'][:, 1]) > 1e-5
    assert np.isfinite(model['K']).all()
