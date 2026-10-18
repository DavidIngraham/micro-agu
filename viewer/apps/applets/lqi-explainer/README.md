# LQI teaching applets

Views: coupling, motion, observer, online, offline, gains, replay. All use the shared applet lifecycle. No autonomous animation; touch/keyboard controls work without hover. Diagrams use native HTML/SVG/canvas rather than loading the simulator.

`deployed-gains.h` is copied from ArduPilot revision 20a8fb59c8c050848a5dd0b2af24eaca062f5d2a, verified against `viewer/assets/runtime/build.json`. The source is GPL-3.0-or-later, like the runtime. `generate.py` reconstructs the selected normalized LQI and asserts agreement with K, then exports the teaching data. Run from any directory with `simulation/.venv/bin/python viewer/apps/applets/lqi-explainer/generate.py`. NumPy/SciPy and the archived longitudinal campaign runs are required (extract that campaign's raw-runs.tar.gz if necessary). The site uses committed data.json and needs no Python at runtime.

Coupling and observer demonstrations use the released local A/B/L matrices, not nonlinear flight truth. The observer has synthetic deterministic measurement noise and an initially unobserved relative-pitch perturbation. This intentionally illustrates prediction/correction without claiming real-world estimator accuracy. Coupling uses a +1 percentage-point pulse from 1 to 1.5 s. Raw state components are labeled; body up velocity is not earth climb.

Gain cells show -100 US K / physical-state-scale times the displayed example deviation. Integral normalization is 1 m, 10 deg/s, 0.5 m/s. A unit turn integral therefore corresponds to 10 degrees of accumulated rate error. Rotor state scaling is 1000 rpm. The model's canopy rates are absolute angular rates expressed in the payload frame, not relative rates. Motion demonstration holds payload rate zero explicitly.

The replay reduces heldout-0-lqi/baseline.json to 2 Hz. Relative pitch rate is canopy minus payload angular velocity expressed in the payload frame; altitude error uses logged altitude minus target. Provenance records original JSON hashes and test specifications. Full outcome metrics remain in the existing release-evidence applet; plotted downsampling must not be used to recompute peak metrics.
