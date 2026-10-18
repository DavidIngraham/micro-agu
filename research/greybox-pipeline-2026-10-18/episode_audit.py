"""Longer-episode spectral/timing audit plus two unused manual episodes.

Candidates stay frozen; none of these measurements influence selection.
"""

import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from microagu_sim.sysid.data import load_session
from microagu_sim.sysid.metrics import metrics, response_residual
from microagu_sim.sysid.model import DEFAULT, replay
from scipy.signal import csd, welch

OUT = Path(__file__).resolve().parent
EPISODES = [
    ("manual-266", 266.64, 339.36, "training"),
    ("manual-371", 371.52, 420.84, "previous_validation"),
    ("manual-435", 435.64, 468.60, "previous_validation"),
    ("manual-529", 529.84, 578.92, "training"),
    ("manual-1985", 1985.28, 2018.68, "training"),
    ("manual-903", 903.80, 926.12, "unused_check"),
    ("manual-1087", 1087.20, 1106.00, "unused_check"),
]


def save(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def spectral(observed, predicted, commands, nperseg):
    a, m, u = observed[50:-10], predicted[50:-10], commands[50:-10]
    output = {}

    def density(x, y=None):
        return (
            welch(x, fs=25, nperseg=nperseg, noverlap=nperseg // 2)
            if y is None
            else csd(x, y, fs=25, nperseg=nperseg, noverlap=nperseg // 2)
        )

    for name, command, axis in [
        ("differential_yaw", u[:, 2] - u[:, 1], 2),
        ("common_pitch", u[:, 1:3].mean(axis=1), 1),
    ]:
        f, po = density(a[:, axis])
        _, pm = density(m[:, axis])
        _, pu = density(command)
        _, co = density(command, a[:, axis])
        _, cm = density(command, m[:, axis])
        mask = (f >= 1.2) & (f <= 1.9)
        idx = np.flatnonzero(mask)[np.argmax(po[mask])]
        observed_coherence = abs(co) ** 2 / np.maximum(pu * po, 1e-20)
        model_coherence = abs(cm) ** 2 / np.maximum(pu * pm, 1e-20)
        df = f[1] - f[0]
        output[name] = {
            "peak_hz": float(f[idx]),
            "observed_phase_deg": float(np.rad2deg(np.angle(co[idx]))),
            "model_phase_deg": float(np.rad2deg(np.angle(cm[idx]))),
            "phase_error_deg": float(np.rad2deg(np.angle(cm[idx] * np.conj(co[idx])))),
            "observed_coherence": float(observed_coherence[idx]),
            "model_coherence": float(model_coherence[idx]),
            "observed_rms": float(np.sqrt(po[mask].sum() * df)),
            "model_rms": float(np.sqrt(pm[mask].sum() * df)),
            "command_band_rms_mm": float(np.sqrt(pu[mask].sum() * df)),
            "frequencies": f[mask].tolist(),
            "phase_error_band_deg": np.rad2deg(
                np.angle(cm[mask] * np.conj(co[mask]))
            ).tolist(),
            "observed_coherence_band": observed_coherence[mask].tolist(),
        }
    return {
        "nperseg": nperseg,
        "welch_averages": 1 + (len(a) - nperseg) // (nperseg // 2),
        "note": "Overlapping averages are not independent; coherence is descriptive under feedback.",
        "channels": output,
    }


def evaluate(job):
    s, theta, family, kind = job
    predicted, _, _ = replay(s, theta)
    observed = np.rad2deg(s.observation[:, 3:6])
    r = response_residual(observed, predicted)
    return dict(
        episode=s.episode,
        kind=kind,
        family=family,
        duration_s=float(s.time[-1] - s.time[0]),
        response_loss=float(r @ r),
        **metrics(observed, predicted),
        spectral=[spectral(observed, predicted, s.commands, n) for n in [128, 256]],
    )


def main():
    # Declare added checks before reading their outputs. No fitting occurs here.
    save(
        "episode-protocol.json",
        {
            "episodes": EPISODES,
            "nperseg": [128, 256],
            "selection": "Reuse frozen candidates only; no refit or new selection",
            "new_checks": "903.80..926.12 and 1087.20..1106.00 s: manual episodes excluded from this comparison fitting/selection and earlier 20 s gradient validation; broader historical studies may have examined them",
            "evidence": "Additional episodes in the same historical flight, not independent prospective data",
        },
    )
    base = json.loads((OUT / "session.json").read_text())["segments"][0]
    cfg = {
        "schema_version": 1,
        "segments": [
            dict(
                base,
                name=name,
                episode=name,
                role="train",
                start_s=start,
                duration_s=round(end - start, 2),
            )
            for name, start, end, _ in EPISODES
        ],
    }
    save("episode-session.json", cfg)
    _, segments, _ = load_session(OUT / "episode-session.json")
    choices = json.loads((OUT / "frozen-selection.json").read_text())
    jobs = [
        (s, theta, family, kind)
        for s, (_, _, _, kind) in zip(segments, EPISODES)
        for family, theta in [
            ("baseline", DEFAULT),
            *[(k, v["theta"]) for k, v in choices.items()],
        ]
    ]
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(evaluate, jobs))
    save("episode-audit.json", rows)
    # Clock-age audit directly from the causal input, independent of model output.
    a = np.load(base["causal_input"])
    raw = json.loads(
        (
            Path(base["causal_input"]).parent.parent / "2026-02-15 15-38-50/MODE.json"
        ).read_text()
    )
    mt = np.array([v["TimeUS"] / 1e6 for v in raw])
    modes = np.array([v["ModeNum"] for v in raw])
    clock = []
    for name, start, end, _ in EPISODES:
        mask = (a["t"] >= start - 1e-5) & (a["t"] <= end + 1e-5)
        assert (modes[np.searchsorted(mt, a["t"][mask], side="right") - 1] == 0).all()
        channels = {}
        for source in ["ATT", "IMU", "XKF1", "RCOU"]:
            age = a["t"][mask] - a[source + "_source_time"][mask]
            channels[source] = {
                "median_s": float(np.median(age)),
                "p95_s": float(np.quantile(age, 0.95)),
                "max_s": float(age.max()),
                "min_s": float(age.min()),
            }
        clock.append(
            {"episode": name, "valid_samples": int(mask.sum()), "source_age": channels}
        )
    save("source-clock-audit.json", clock)
    print(
        json.dumps(
            [
                {
                    "episode": r["episode"],
                    "family": r["family"],
                    "response_loss": r["response_loss"],
                    "phase": r["spectral"][0]["channels"]["differential_yaw"][
                        "phase_error_deg"
                    ],
                    "coherence": r["spectral"][0]["channels"]["differential_yaw"][
                        "observed_coherence"
                    ],
                }
                for r in rows
            ],
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
