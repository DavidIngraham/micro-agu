"""Frequency, robustness, finite-horizon state diagnostics and truth-state benchmark."""
import json
import numpy as np
from scipy.linalg import expm, logm, solve_discrete_are
from scipy.spatial.transform import Rotation as R
from study import HERE, DT, BASE, XS, US, NAMES, plants, controller, output_matrix, save
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLORS = ['#286b89', '#c36535', '#668a43']
plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})

def lqi(p, move_penalty=100., integral_weight=.08, rate_weight=1., state_weight=.002):
    """Full-state discrete LQI, penalizing command increments and absolute commands.
    No observer: truth-state benchmark, not a proposed flight implementation.
    """
    A = p['A']*XS[None, :]/XS[:, None]; B = p['B']*US[None, :]/XS[:, None]
    E = p['E']/XS[:, None]; C = p['C']*XS[None, :]
    AA = np.zeros((20, 20)); BB = np.zeros((20, 2)); WW = np.zeros((20, 3))
    AA[:16, :16] = A; AA[:16, 16:18] = B; AA[16:18, 16:18] = np.eye(2)
    BB[:16] = B; BB[16:18] = np.eye(2); WW[:16] = E
    AA[18:, :16] = DT*C[[5, 3]]/np.array([1., 10.])[:, None]
    AA[18:, 18:] = np.eye(2)
    Y = np.zeros((7, 20)); Y[:5, :16] = C[[5, 3, 0, 1, 2]]/np.array([1, 10, .5, np.deg2rad(30), np.deg2rad(30)])[:, None]
    Y[5:, 16:18] = np.eye(2)
    Y[3:5] *= np.sqrt(rate_weight)
    Q = Y.T@Y+np.diag(np.r_[np.full(16, state_weight), np.zeros(2), integral_weight, integral_weight])
    RR = np.eye(2)*move_penalty
    P = solve_discrete_are(AA, BB, Q, RR)
    K = np.linalg.solve(RR+BB.T@P@BB, BB.T@P@AA)
    F = AA-BB@K; G = np.zeros((20, 2)); G[18:] = -DT*np.diag([1., .1])
    U = -K; U[:, 16:18] += np.eye(2); U = US[:, None]*U
    return F, G, WW, U, K

def robust(p, cfg, gain, delays, rotor):
    pp = dict(p)
    # Change rotor local time scale while preserving the nominal DC relation.
    if rotor != 1:
        aug = np.zeros((18, 18)); aug[:16, :16] = p['A']; aug[:16, 16:] = p['B']; aug[16:, 16:] = np.eye(2)
        continuous = logm(aug).real/DT; continuous[14] *= rotor
        discrete = expm(continuous*DT); pp['A'] = discrete[:16, :16]; pp['B'] = discrete[:16, 16:]
    F, _, _, U, _ = controller(pp, cfg)
    B = pp['B']*np.array(gain)[None, :]
    n = sum(delays); out = np.zeros((24+n, 24+n))
    out[:24, :24] = F; out[:16, :24] -= pp['B']@U
    pos = 24
    for j, delay in enumerate(delays):
        if delay:
            out[:16, pos+delay-1] = B[:, j]
            out[pos, :24] = U[j]
            for k in range(1, delay): out[pos+k, pos+k-1] = 1
            pos += delay
        else: out[:16, :24] += np.outer(B[:, j], U[j])
    return float(np.log(max(abs(np.linalg.eigvals(out))))/DT)

def main():
    ps = plants(); p = ps[0]
    config = json.loads((HERE/'optimization.json').read_text()); candidate = config['candidate']
    configs = {'baseline': BASE, 'candidate': candidate}
    freqs = np.geomspace(.01, 20, 500); curves = {}; agreement = {}
    for name, cfg in configs.items():
        F, G, W, U, Ur = controller(p, cfg); Z = F[16:]
        ref, gust, sensitivity, offdiag, diagonal = [], [], [], [], []
        max_error = 0.
        for f in freqs:
            z = np.exp(2j*np.pi*f*DT)
            plant = np.linalg.solve(z*np.eye(16)-p['A'], p['B'])
            K = -(U[:, :16]+U[:, 16:]@np.linalg.solve(z*np.eye(8)-Z[:, 16:], Z[:, :16]))
            loop = K@plant
            S = np.linalg.inv(np.eye(2)+loop)
            Sstate = np.eye(2)+U@np.linalg.solve(z*np.eye(24)-F, np.vstack((p['B'], np.zeros((8, 2)))))
            max_error = max(max_error, np.linalg.norm(S-Sstate))
            Sn = S*US[None, :]/US[:, None]
            sensitivity.append(np.linalg.svd(Sn, compute_uv=False)[0])
            diagonal.append(max(abs(np.diag(Sn))))
            H = np.c_[p['C'], np.zeros((6, 8))]@np.linalg.solve(z*np.eye(24)-F, G)
            ref.append([H[5, 0], H[3, 1]])
            Hg = np.c_[p['C'], np.zeros((6, 8))]@np.linalg.solve(z*np.eye(24)-F, W)
            gust.append([Hg[0, 2], Hg[1, 2], Hg[3, 1]])
            offdiag.append(abs(loop[0, 1])+abs(loop[1, 0]))
        curves[name] = (np.array(ref), np.array(gust), np.array(sensitivity))
        agreement[name] = dict(max_absolute_matrix_difference=float(max_error),
                              peak_actuator_sensitivity=float(max(sensitivity)),
                              peak_diagonal_sensitivity=float(max(diagonal)),
                              peak_frequency_hz=float(freqs[np.argmax(sensitivity)]))
        assert max_error < 1e-7
    save('frequency.json', agreement)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    for color, (name, (ref, gust, sens)) in zip(COLORS, curves.items()):
        ax[0, 0].semilogx(freqs, 20*np.log10(abs(ref[:, 0])), label=name, color=color)
        ax[0, 1].semilogx(freqs, 20*np.log10(abs(ref[:, 1])), label=name, color=color)
        ax[1, 0].semilogx(freqs, abs(gust[:, 1])*180/np.pi, label=name, color=color)
        ax[1, 1].semilogx(freqs, sens, label=name, color=color)
    for a, title, ylabel in zip(ax.flat, ['Altitude command tracking', 'Heading-rate command tracking',
                                        'Vertical wind → payload pitch rate', 'Coupled actuator sensitivity'],
                               ['Magnitude (dB)', 'Magnitude (dB)', '(deg/s) / (m/s)', 'Largest singular value']):
        a.set(title=title, xlabel='Frequency (Hz)', ylabel=ylabel); a.grid(alpha=.2); a.legend()
    fig.suptitle('Two descriptions of the same sampled linear system agree numerically')
    fig.savefig(HERE/'frequency.png', dpi=160); plt.close(fig)

    rows = []
    for pp in ps:
        for name, cfg in configs.items():
            for gt in [.75, 1.25]:
                for gb in [.75, 1.25]:
                    for dt in [0, 2, 4]:
                        for db in [0, 2, 4]:
                            for rotor in [.7, 1., 1.3]:
                                pole = robust(pp, cfg, [gt, gb], [dt, db], rotor)
                                rows.append(dict(case=pp['name'], controller=name, throttle_gain=gt,
                                                 brake_gain=gb, throttle_delay_ms=dt*20, brake_delay_ms=db*20,
                                                 rotor_rate=rotor, max_real=pole))
    save('robustness.json', rows)
    fig, ax = plt.subplots(figsize=(10, 5), layout='constrained')
    summary = {}
    for i, (name, color) in enumerate(zip(configs, COLORS)):
        worst = [max(r['max_real'] for r in rows if r['case']==pp['name'] and r['controller']==name) for pp in ps]
        ax.bar(np.arange(len(ps))+(i-.5)*.36, worst, width=.36, label=name, color=color)
        rr = [r for r in rows if r['controller']==name]
        summary[name] = dict(total=len(rr), unstable=sum(r['max_real']>=0 for r in rr), worst=max(rr, key=lambda r:r['max_real']))
    ax.axhline(0, color='black', lw=1); ax.set_xticks(range(len(ps)), [p['name'].replace('_', '\n') for p in ps])
    ax.set(ylabel='Worst pole real part (1/s)', title='Combined gain, rotor-rate and unequal command-delay stress grid')
    ax.legend(); ax.grid(axis='y', alpha=.2); fig.savefig(HERE/'robustness.png', dpi=160); plt.close(fig)
    save('robustness-summary.json', summary)

    # Finite-horizon Gramians remain meaningful with the altitude integrator.
    # Known model, zero/known wind; independent illustrative measurement errors.
    An = p['A']*XS[None, :]/XS[:, None]; Bn = p['B']*US[None, :]/XS[:, None]
    # Payload attitude, body gyro, ground velocity and height. Velocity derivative
    # includes attitude changes (body-coordinate states are not GNSS outputs).
    pitch = np.zeros((2, 16)); pitch[0, 0]=1; pitch[1, 1]=1
    gyro = np.zeros((3, 16)); gyro[:, 8:11]=np.eye(3)
    from study import Chart, setup
    # Analytic GNSS velocity Jacobian via the saved trim attitude and chart convention.
    rp = R.from_quat(p['state'][3:7]); v0=rp.inv().apply(p['state'][7:10]); e0=rp.as_euler('xyz')
    vel = np.zeros((3,16)); vel[:,5:8]=rp.as_matrix()
    for j in range(2):
        e=np.zeros(3);e[j]=1e-6
        vel[:,j]=(R.from_euler('xyz',e0+e).apply(v0)-R.from_euler('xyz',e0-e).apply(v0))/2e-6
    h = np.zeros((1,16));h[0,15]=1
    suite=np.vstack((pitch/np.deg2rad(1),gyro/np.deg2rad(1),vel/.1,h/.2))*XS
    canopy=np.zeros((3,16)); canopy[:,11:14]=np.eye(3)/np.deg2rad(1); canopy*=XS
    sensors={'payload gyro':gyro/np.deg2rad(1)*XS,
             'payload + navigation':suite,
             '+ canopy gyro':np.vstack((suite,canopy))}
    diagnostics={}; wc=np.zeros((16,16)); power=np.eye(16)
    wo={key:np.zeros((16,16)) for key in sensors}
    for _ in range(round(4/DT)):
        wc += power@Bn@Bn.T@power.T/DT
        for key,M in sensors.items():wo[key] += DT*(M@power).T@(M@power)
        power=An@power
    for key,mat in wo.items():
        eig=np.linalg.eigvalsh((mat+mat.T)/2)
        diagnostics[key]=dict(eigenvalues=eig.tolist(), positive_condition=float(eig[-1]/eig[0]) if eig[0]>1e-10 else None)
    eig,V=np.linalg.eigh((wc+wc.T)/2); weak=V[:,0]
    diagnostics['control']=dict(eigenvalues=eig.tolist(),weakest_direction=dict(zip(NAMES,weak.tolist())))
    # Modal input coupling and output visibility: normalize each right eigenvector.
    values,V=np.linalg.eig(An); V/=np.linalg.norm(V,axis=0); left=np.linalg.inv(V)
    modes=[]
    for j,z in enumerate(values):
        pole=np.log(complex(z))/DT
        if pole.imag < 1e-6:continue
        part=abs(V[:,j]*left[j]);part/=sum(part)
        ii=np.argsort(part)[-3:][::-1]
        modes.append(dict(hz=float(pole.imag/2/np.pi), real=float(pole.real),
                          participation={NAMES[k]:float(part[k]) for k in ii},
                          input_coupling=abs(left[j]@Bn).tolist(),
                          visibility={k:float(np.linalg.norm(M@V[:,j])) for k,M in sensors.items()}))
    diagnostics['modes']=modes
    modes.sort(key=lambda m:m['hz'])
    # Asking the observer to estimate constant wind is a different problem from
    # observing the known-wind plant. No force/moment bias states are included.
    Aw=np.eye(19);Aw[:16,:16]=An;Aw[:16,16:]=p['E']*np.array([.5,.5,.25])/XS[:,None]
    wind_obs={}
    for key,M in sensors.items():
        Mw=np.c_[M,np.zeros((len(M),3))];power=np.eye(19);gram=np.zeros((19,19))
        for _ in range(round(4/DT)):
            gram+=DT*(Mw@power).T@(Mw@power);power=Aw@power
        ev=np.linalg.eigvalsh((gram+gram.T)/2)
        wind_obs[key]=ev.tolist()
    diagnostics['constant_wind_augmented_observability']=wind_obs
    diagnostics['assumptions']='4 s horizon; scaled coordinates; known model/wind; assumed independent sensor errors, not estimator covariance.'
    save('state-diagnostics.json',diagnostics)
    fig, ax=plt.subplots(1,2,figsize=(12,5),layout='constrained')
    for name,color in zip(sensors,COLORS):
        ax[0].semilogy(range(1,17),np.maximum(diagnostics[name]['eigenvalues'],1.),'o-',label=name,color=color)
    ax[0].annotate('Gyro-only altitude: zero\n(shown at plot floor)',xy=(1,1),xytext=(3,1.5),arrowprops=dict(arrowstyle='->'),fontsize=9)
    ax[0].set(title='What measurements reveal over four seconds',xlabel='Gramian eigenvalue, weakest → strongest',ylabel='Scaled observability');ax[0].legend(fontsize=9)
    for j,label in enumerate(['Throttle','Differential brake']):
        ax[1].semilogy([m['hz'] for m in modes],[m['input_coupling'][j] for m in modes],'o-',label=label)
    ax[1].set(title='Which actuator reaches each oscillatory mode?',xlabel='Mode frequency (Hz)',ylabel='Normalized modal input coupling');ax[1].legend()
    for aa in ax:aa.grid(alpha=.2)
    fig.suptitle('State information adds actuator/sensor structure; scales are declared assumptions')
    fig.savefig(HERE/'state-diagnostics.png',dpi=160);plt.close(fig)

    F,G,W,U,K=lqi(p);np.savez_compressed(HERE/'oracle.npz',F=F,G=G,W=W,U=U,K=K)
    save('oracle.json',dict(move_penalty=100.,max_real=float(np.log(max(abs(np.linalg.eigvals(F))))/DT),
                            note='Full-state LQI, normalized input increments; nonlinear validation must enforce limits.'))
    print('frequency',agreement,'robustness',summary,flush=True)

if __name__=='__main__':main()
