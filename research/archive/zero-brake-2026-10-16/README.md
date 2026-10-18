# Zero-trim, one-sided common brake tuning

AI-assisted implementation and analysis. The physical common-brake trim is exactly
zero; differential steering still pulls one brake. The existing allocator clips
common demand to [0, authority] and reports applied commands to both observer and
coupled integral anti-windup. Negative requests are retained as allocation residuals,
not silently clipped before anti-windup. There is no new negative actuator authority.

The synthesis fixes common brake as a trim constraint and uses the right-sided
input derivative at zero. Unconstrained eigenvalues are only a local synthesis
check, not a stability claim for the constrained controller.

## Planned comparison

- Iteration 1: zero common trim, original input costs and pitch-rate weights.
- Iteration 2: common-brake effort 10000 (10x original), pitch-rate weight 60
  (2x original). Reduces settled braking but rejected for 61 m AUTO cross-track RMS.
- Iteration 3: common-brake effort 3000, original pitch-rate weight 30. Tests
  a smaller effort increase without changing the pitch-rate cost.
- Tune against prior cases 00 (AUTO), 01 and 15 (oscillation regressions), and
  03 and 09 (inefficient settled CRUISE). Keep all trial results, including failures.
- Use identical existing flight-log calibration, 132 mm geometry, weather seeds,
  disturbances and 180-second evaluation protocol. No model retuning.
- A candidate is competitive enough for the full campaign if every tuning run
  completes, the two oscillation cases recover below 0.2 degrees late pitch AC RMS,
  and median tracking RMS is within 25% of current LQI in each task. Power is a
  separate selection metric; lower power alone cannot excuse poor damping.
- If competitive, rerun all 16 paired cases, including baseline PG+TECS. This is
  a regression/sensitivity comparison; tuning cases overlap it, so it is not an
  independent validation set or an estimate of field failure probability.

`run.py iteration-1` runs the five tuning cases with the current packaged firmware.
`run.py campaign --cases all --both` runs the full paired campaign.
Firmware hashes and the exact sample harness are saved beside each iteration.
New runs also retain rotor RPM and voltage; old runs only support quasi-steady
power reconstruction, so like-for-like comparisons continue to use that estimate.
The original packaged firmware is backed up under `/tmp/microagu-zero-brake/original`.

## Candidate decision before full campaign

Iteration 1 was retained. Iterations 2 and 3 were rejected for AUTO cross-track
RMS of 61.0 m and 43.9 m, respectively, despite reducing settled common brake.
Iteration 1 retained recovery (late pitch RMS below 0.05 degrees in both known
oscillation cases), improved AUTO altitude RMS from 3.95 to 3.48 m, and changed
AUTO cross-track RMS from 3.36 to 3.58 m. CRUISE median altitude RMS rose 24.5%,
and cross-track RMS rose from 0.334 to 0.446 m. The latter misses the initial
25% relative screen, but is only 0.112 m in absolute terms. The full campaign is
therefore a deliberately documented borderline-candidate investigation, not a
claim that every predeclared threshold passed. No rejected tune is deployed.

The selected synthesis command is:

```sh
simulation/.venv/bin/python research/archive/legacy-control/design_mimo.py \
  --output simulation/data/control/mimo --common-trim 0 \
  --common-effort 1000 --pitch-weight 30
```
