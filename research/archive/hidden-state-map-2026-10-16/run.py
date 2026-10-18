"""Three-stage conditional hidden-state/damping experiment on existing flights."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
from concurrent.futures import ProcessPoolExecutor
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.sparse.linalg import lsmr

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


est = load('hidden_estimation', ROOT/'control/src/microagu_control/estimation.py')
prior = load('wind_calibration', HERE.parent/'wind-calibration-2026-10-16/run.py')


def segment(flight, start, duration=10):
    a, b, _ = prior.inputs(flight)
    sim, _ = prior.prior.migrated_model(flight, 'routed', (1., 1., .0743/.066))
    i = int(np.argmin(abs(a['t']-start)))
    sl = slice(i, i+round(duration/.04)+1)
    assert np.all(a['valid'][sl]), 'Invalid source data'
    y = prior.study.init(sim, a, b, i, .35)
    rel = est.hidden_from_state(y)[0, :3]
    commands = np.array([sim.controls(*u) for u in np.column_stack([a['U'][sl, 0], a['brake_mm'][sl]])])
    return est.Segment(f'{flight}-{start}', a['t'][sl].copy(), a['Y'][sl].copy(), commands,
                       b['rpm'][sl].copy(), b['voltage'][sl].copy(), dict(sim.parameters),
                       sim.geometry['mass'], sim.geometry['imu'], sim.geometry['hinge'],
                       rel, a['wind'][i, :2])


def metadata(result, graph, elapsed):
    x = result.x
    return dict(cost=float(result.cost), success=bool(result.success), status=int(result.status),
                message=result.message, nfev=int(result.nfev), optimality=float(result.optimality),
                elapsed_s=elapsed, damping=graph.damping_values(x).tolist(),
                active_bounds=int(np.count_nonzero(result.active_mask)),
                rejected_trial_edges=graph.rejected_edges,
                residual_rms=float(np.sqrt(np.mean(result.fun**2))))


def solve_save(name, graph, x=None, max_nfev=80, tolerance=2e-4):
    t0 = time.perf_counter()
    result = graph.solve(x, max_nfev=max_nfev, tolerance=tolerance)
    info = metadata(result, graph, time.perf_counter()-t0)
    np.savez_compressed(HERE/f'{name}.npz', x=result.x, residual=result.fun,
                        damping=graph.damping_values(result.x))
    (HERE/f'{name}.json').write_text(json.dumps(info, indent=2)+'\n')
    print(name, json.dumps(info), flush=True)
    return result


def diagnostic_job(kind):
    segments = training()
    graph = est.Graph(segments, fit_damping=True)
    if kind == 'strict':
        x = np.load(HERE/'joint-nominal.npz')['x']
    else:
        x = np.r_[np.concatenate([np.load(HERE/f'fixed-{s.name}-nominal.npz')['x'] for s in segments]),
                  np.log([float(kind), float(kind)])]
    solve_save(f'diagnostic-{kind}', graph, x, max_nfev=120,
               tolerance=2e-6 if kind == 'strict' else 2e-4)


def stage1_job(job):
    flight, start, variant = job
    s = segment(flight, start)
    settings = dict(nominal=(.10, 1., 0.), start_low=(.10, 1., -.10),
                    start_high=(.10, 1., .10), slow_wind=(.03, 1., 0.),
                    loose_model=(.10, 2., 0.))
    diffusion, discrepancy, offset = settings[variant]
    graph = est.Graph([s], wind_diffusion=diffusion, discrepancy=discrepancy)
    name = f'fixed-{s.name}-{variant}'
    result = solve_save(name, graph, graph.initial(offset))
    z = result.x.reshape(-1, 8)
    np.savez_compressed(HERE/f'augmented-{s.name}-{variant}.npz',
                        time=s.time[s.knots], original_observation=s.observation[s.knots],
                        relative_rotvec=z[:, :3], relative_angular_velocity=z[:, 3:6],
                        wind_ned=np.column_stack([z[:, 6:8], np.zeros(len(z))]),
                        residual=result.fun)
    return name


def training():
    return [segment(f, t) for f, (_, starts) in prior.study.FLIGHTS.items() for t in starts[::2]]


def stage2(variant='nominal'):
    segments = training()
    settings = dict(nominal=(.10, 1.), slow_wind=(.03, 1.), loose_model=(.10, 2.))
    diffusion, discrepancy = settings[variant]
    graph = est.Graph(segments, fit_damping=True, wind_diffusion=diffusion, discrepancy=discrepancy)
    x = np.r_[np.concatenate([np.load(HERE/f'fixed-{s.name}-{variant}.npz')['x'] for s in segments]), np.log([3., 3.])]
    result = solve_save(f'joint-{variant}', graph, x, max_nfev=120)
    # Local conditional information after allowing all hidden states to move.
    # Remove the explicit damping prior rows, but retain state/wind priors.
    # The robust solver Jacobian is locally weighted, not a calibrated covariance.
    jac = result.jac[:-2]
    nuisance, parameter = jac[:, :-2], jac[:, -2:].toarray()
    projected = []
    diagnostics = []
    for column in parameter.T:
        projection = lsmr(nuisance, column, atol=1e-7, btol=1e-7, maxiter=3000)
        projected.append(column-nuisance@projection[0])
        diagnostics.append(dict(stop=int(projection[1]), iterations=int(projection[2])))
    projected = np.column_stack(projected)
    information = projected.T@projected
    covariance = np.linalg.pinv(information)
    info = dict(projected_singular_values=np.linalg.svd(projected, compute_uv=False).tolist(),
                conditional_log_damping_std=np.sqrt(np.diag(covariance)).tolist(),
                correlation=float(covariance[0, 1]/np.sqrt(covariance[0, 0]*covariance[1, 1])),
                projection_diagnostics=diagnostics,
                caveat='Local conditional curvature only; ignores navigation correlation and model bias, not calibrated uncertainty.')
    (HERE/f'information-{variant}.json').write_text(json.dumps(info, indent=2)+'\n')
    return result


def metrics(pred, truth, imu, discard=0):
    obs = est.observe(pred, imu)[discard:]
    target = truth[discard:]
    wrap = lambda a: np.arctan2(np.sin(a), np.cos(a))
    error = np.column_stack([np.rad2deg(wrap(obs[:, :3]-target[:, :3])),
                             np.rad2deg(obs[:, 3:6]-target[:, 3:6]), obs[:, 6:9]-target[:, 6:9]])
    return dict(zip(['roll', 'pitch', 'yaw', 'roll_rate', 'pitch_rate', 'yaw_rate', 'vn', 've', 'vd'],
                    np.sqrt(np.mean(error**2, axis=0)).tolist()))


def check_job(job):
    flight, start, scheme = job
    s = segment(flight, start-10, 40)
    prefix = segment(flight, start-10, 10)
    damping = [3., 3.] if scheme == 'fixed' else np.load(HERE/'joint-nominal.npz')['damping']
    graph = est.Graph([prefix], damping=damping)
    result = solve_save(f'prefix-{flight}-{start}-{scheme}', graph)
    hidden = result.x.reshape(-1, 8)[-1]
    forecast = est.predict(s, hidden, damping, 250, 1000)
    baseline_hidden = np.r_[est.hidden_from_state(prior.study.init(
        prior.prior.migrated_model(flight, 'routed', (1., 1., .0743/.066))[0],
        *prior.inputs(flight)[:2], int(np.argmin(abs(prior.inputs(flight)[0]['t']-start))), .35))[0],
        prior.inputs(flight)[0]['wind'][int(np.argmin(abs(prior.inputs(flight)[0]['t']-start))), :2]]
    baseline = est.predict(s, baseline_hidden, damping, 250, 1000)
    # Same estimated endpoint wind, varying only canopy initialization.
    perturbations = []
    for offset in [-.1, .1]:
        h = hidden.copy(); h[1] += offset
        perturbations.append(est.predict(s, h, damping, 250, 1000))
    actual = s.observation[250:]
    rows = {label: {str(cut): metrics(y[:, :26], actual, s.imu, round(cut/.04))
                    for cut in [0, 2, 5, 10]} for label, y in [('inferred', forecast), ('historical', baseline)]}
    delta = np.stack([est.observe(y[:, :26], s.imu)[:, 3:6]-est.observe(forecast[:, :26], s.imu)[:, 3:6]
                      for y in perturbations])
    rows['initialization_rate_spread_deg_s'] = {
        f'{lo}-{hi}': float(np.sqrt(np.mean(np.rad2deg(delta[:, round(lo/.04):round(hi/.04)])**2)))
        for lo, hi in [(0, 2), (2, 5), (5, 10), (10, 20), (20, 30)]}
    np.savez_compressed(HERE/f'forecast-{flight}-{start}-{scheme}.npz', time=s.time[250:],
                        prediction=forecast, historical=baseline, observation=actual,
                        initial_hidden=hidden, damping=damping)
    (HERE/f'check-{flight}-{start}-{scheme}.json').write_text(json.dumps(rows, indent=2)+'\n')
    print('checked', flight, start, scheme, flush=True)
    return rows


def synthetic(seed=0, fit=False, mismatch=False, varying=False):
    s = segment('trout_lake', 600, 10)
    t = s.time-s.time[0]
    # Excitation on both brake channels; use legal kernel units already mapped.
    s.commands[:, 1] += .008*(1+np.sin(1.7*t))
    s.commands[:, 2] += .008*(1+np.sin(1.1*t+.8))
    true_damping = np.array([2., 4.])
    h = np.r_[s.initial_relative+[.03, -.06, .04], [.03, -.05, .04], [1.2, -.6]]
    source = est.predict(s, h, true_damping, 0, len(t)-1)
    wind_truth = np.tile(h[6:8], (len(t), 1))
    if varying:
        wind_truth += np.column_stack([.3*np.sin(t*.6), .2*np.sin(t*.4)])
        kernel = est.Graph([s], damping=true_damping).kernels(true_damping)[0]
        for j in range(len(t)-1):
            source[j+1] = kernel.replay_numpy(np.ascontiguousarray(source[j, :26]),
                np.ascontiguousarray(s.commands[j:j+2].ravel()), np.r_[wind_truth[j], 0.],
                float(s.voltage[j]), float(source[j, 26]), .04, 32, True)[-1]
    rng = np.random.default_rng(seed)
    s.observation = est.observe(source[:, :26], s.imu)
    s.observation += rng.normal(size=s.observation.shape)*np.r_[[.003]*3, [.005]*3, [.03]*3]
    s.rpm = source[:, 26]
    # Change physics only in estimator, not in generated truth.
    if mismatch:
        s.parameters['CLbrake'] *= 1.3
    graph = est.Graph([s], damping=[3., 3.] if fit else true_damping, fit_damping=fit)
    name = f'synthetic-{seed}-'+('joint' if fit else 'fixed')+('-mismatch' if mismatch else '')+('-varying' if varying else '')
    result = solve_save(name, graph, max_nfev=120)
    z = result.x[:graph.nuisance_size].reshape(-1, 8)
    actual = est.hidden_from_state(source[s.knots, :26])
    angle = (R.from_rotvec(actual[:, :3]).inv()*R.from_rotvec(z[:, :3])).magnitude()
    stats = dict(wind_rmse=float(np.sqrt(np.mean((z[:, 6:8]-wind_truth[s.knots])**2))),
                 angle_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(angle**2)))),
                 relative_rate_rmse=float(np.sqrt(np.mean((z[:, 3:6]-actual[:, 3:6])**2))),
                 true_damping=true_damping.tolist(), fitted_damping=graph.damping_values(result.x).tolist())
    (HERE/f'{name}-truth.json').write_text(json.dumps(stats, indent=2)+'\n')
    np.savez_compressed(HERE/f'{name}-truth.npz', time=s.time[s.knots], hidden_truth=actual,
                        wind_truth=wind_truth[s.knots], hidden_estimate=z)
    print(name, stats, flush=True)
    return stats


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=['smoke', 'fixed', 'joint', 'check', 'check-fixed', 'check-joint', 'synthetic', 'varying', 'diagnostic'])
    parser.add_argument('--flight', choices=list(prior.study.FLIGHTS))
    parser.add_argument('--start', type=int)
    args = parser.parse_args()
    if args.stage == 'smoke':
        graph = est.Graph([segment('trout_lake', 600, 2)])
        solve_save('smoke', graph, max_nfev=3)
    elif args.stage == 'fixed':
        jobs = [(f, t, v) for f, (_, starts) in prior.study.FLIGHTS.items() for t in starts[::2]
                for v in ['nominal', 'start_low', 'start_high', 'slow_wind', 'loose_model']]
        with ProcessPoolExecutor(max_workers=4) as pool:
            list(pool.map(stage1_job, jobs))
    elif args.stage == 'joint':
        with ProcessPoolExecutor(max_workers=3) as pool:
            list(pool.map(stage2, ['nominal', 'slow_wind', 'loose_model']))
    elif args.stage in ['check', 'check-fixed', 'check-joint']:
        schemes = ['fixed', 'joint'] if args.stage == 'check' else [args.stage.split('-')[1]]
        jobs = [(f, t, v) for f, (_, starts) in prior.study.FLIGHTS.items() for t in starts[1::2]
                for v in schemes if (args.flight is None or args.flight == f)
                and (args.start is None or args.start == t)]
        with ProcessPoolExecutor(max_workers=4) as pool:
            list(pool.map(check_job, jobs))
    elif args.stage == 'varying':
        synthetic(0, True, varying=True)
    elif args.stage == 'diagnostic':
        with ProcessPoolExecutor(max_workers=3) as pool:
            list(pool.map(diagnostic_job, ['1', '8', 'strict']))
    else:
        for seed in [0, 1]:
            synthetic(seed, False)
            synthetic(seed, True)
        synthetic(0, True, True)


if __name__ == '__main__':
    main()
