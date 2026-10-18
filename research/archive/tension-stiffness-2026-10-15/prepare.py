"""Build the study's tension law in an isolated Rust wheel, never in production.

Run from the repo root:
uv run --project simulation python research/archive/tension-stiffness-2026-10-15/prepare.py
Exact patch anchors make core refactors fail for review instead of silently
dropping the experimental force law.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / 'simulation'
MODEL = HERE / 'model-rust'


def replace(path, old, new):
    text = path.read_text()
    assert text.count(old) == 1, (path, old)
    path.write_text(text.replace(old, new))


def main():
    MODEL.mkdir(exist_ok=True)
    rust = MODEL / 'source'
    rust.mkdir(exist_ok=True)
    for name in ['Cargo.toml', 'Cargo.lock']:
        shutil.copy2(SOURCE / name, rust / name)
    shutil.copytree(SOURCE / 'crates', rust / 'crates', dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns('__pycache__', '*.so', '*.pyc'))
    core = rust / 'crates/sim-core/src'
    config = core / 'config.rs'
    replace(config, '    pub route_reference_right: f64,', '''    pub route_reference_right: f64,
    pub tension_roll_fraction: f64,
    pub tension_yaw_fraction: f64,
    pub tension_reference: f64,''')
    for anchor in ['            route_reference_right: 0.0,', '            route_reference_right: values[69],']:
        replace(config, anchor, anchor + '''
            tension_roll_fraction: 0.,
            tension_yaw_fraction: 0.,
            tension_reference: 1.,''')
    replace(core/'kernel.rs', '        let parameters = Parameters::from_packed(parameters)?;', '''        let values = array::<73>(parameters)?;
        let mut parameters = Parameters::from_packed(&values[..70])?;
        parameters.tension_roll_fraction = values[70];
        parameters.tension_yaw_fraction = values[71];
        parameters.tension_reference = values[72];''')
    replace(core/'dynamics.rs', '    let reaction = solve.solve(&-mismatch);', (HERE/'tension.rs.inc').read_text())
    with (core/'dynamics.rs').open('a') as f:
        f.write('''
thread_local! {
    pub static LOAD_DIAG: std::cell::Cell<[f64; 8]> = const { std::cell::Cell::new([0.; 8]) };
}
''')
    replace(core/'kernel.rs', '    pub fn brake_paths(', '''    pub fn tension_diagnostic(&self, state: &[f64], command: &[f64], wind: &[f64], rpm: f64, voltage: f64) -> crate::Result<Vec<f64>> {
        use crate::propulsion::*;
        let y = State::from(array::<26>(state)?);
        validate(&y)?;
        let u = array::<3>(command)?;
        let wind = V::from(array::<3>(wind)?);
        let axial = (rotation(&y, 0).transpose() * (vector(&y, 7) - wind)).x;
        let dy = dynamics::rhs(&y, &[thrust(rpm, axial)/self.parameters.thrust_max, u[1], u[2]],
            &self.parameters, self.mass, wind, rpm, (target_rpm(u[0], voltage)-rpm)/MOTOR_TAU_S,
            Some(torque(rpm, axial)));
        if !dy.iter().all(|x| x.is_finite()) { return Err("invalid tension diagnostic".into()); }
        Ok(dynamics::LOAD_DIAG.with(|d| d.get().to_vec()))
    }
    pub fn brake_paths(''')
    replace(rust/'crates/sim-python/src/lib.rs', '    fn brake_paths(', '''    fn tension_diagnostic(&self, state: Vec<f64>, command: Vec<f64>, wind: Vec<f64>, rpm: f64, voltage: f64) -> PyResult<Vec<f64>> {
        self.inner.tension_diagnostic(&state, &command, &wind, rpm, voltage).map_err(err)
    }
    fn brake_paths(''')
    compat = rust/'crates/sim-python/python/microagu_sim/compat'
    replace(compat/'dynamics.py', 'KEYS = _native.parameter_keys()',
            "KEYS = _native.parameter_keys() + ['tension_roll_fraction', 'tension_yaw_fraction', 'tension_reference']")
    with (compat/'dynamics.py').open('a') as f:
        f.write('''
def tension_diagnostic(sim,state,rpm,command,voltage=22.2,wind=(0.,0.,0.)):
 return np.asarray(_kernel(sim.parameters,sim.geometry['mass']).tension_diagnostic(state,sim.controls(*command),wind,float(rpm),float(voltage)))
''')
    env = os.environ.copy()
    env['CARGO_TARGET_DIR'] = str(SOURCE/'target/tension-study')
    wheels = MODEL/'wheels'
    subprocess.run([sys.executable, '-m', 'maturin', 'build', '--release', '--locked',
                    '-m', str(rust/'crates/sim-python/Cargo.toml'), '--out', str(wheels)], env=env, check=True)
    wheel = max(wheels.glob('*.whl'), key=lambda p: p.stat().st_mtime)
    with zipfile.ZipFile(wheel) as archive:
        archive.extractall(MODEL/'python')
    files = [SOURCE/'Cargo.toml', SOURCE/'Cargo.lock', *sorted((SOURCE/'crates').rglob('*.rs'))]
    provenance = dict(source=str(SOURCE), source_commit=subprocess.check_output(
        ['git','rev-parse','HEAD'], cwd=SOURCE, text=True).strip(),
        hashes={str(p.relative_to(SOURCE)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        patch_sha256=hashlib.sha256((HERE/'tension.rs.inc').read_bytes()).hexdigest(),
        wheel_sha256=hashlib.sha256(wheel.read_bytes()).hexdigest())
    (MODEL/'source-provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    print('Built isolated research model:', MODEL)


if __name__ == '__main__':
    main()
