# LQI turn-envelope experiment

Final outcome: all 26 flight simulations completed. Calm-air LQI runs at 25 deg/s
remained well damped with peak altitude error below 0.76 m, but averaged 31.57 deg/s
right and −25.69 deg/s left. Classical runs averaged 25.27 and −24.93 deg/s.
The 25 deg/s LQI tracking gate failed, so testing did not proceed above 25 and
the public 10 deg/s cap remains unchanged. All six disturbed cases also failed
the absolute motion screen at the existing 10 deg/s limit, preventing broad
envelope qualification. These screen failures are not synonymous with instability.

AI-assisted. User requested progressive turn-rate expansion through 15, 20 and
25 deg/s, continuing higher if successful. This study keeps the exact published
`responsive_softspeed` gains, anti-windup matrix, observer, trim, reference maps,
original sensing, 132 mm geometry and physics kernel. No pitot or canopy truth
feeds the controller. EKF remains in the loop.

## Intervention and protocol

The isolated firmware adds `CP3_RMAX` (default 10 deg/s) to replace the hard-coded
turn-reference cap. The existing 40 deg/s² turn-reference slew and 0.7 s climb
filter remain. Research-only `TURNID` telemetry records raw reference, shaped
reference, measured heading rate, estimated/target airspeed, requested/applied
controls, integral states and configured limit. No telemetry changes feedback.

`build.py` temporarily reconstructs the published sources from AP base
`010ada6a4f0b26910fc18ecca176d2e65c5ed9d7`, builds and archives the exact firmware,
then restores all 21 touched source/runtime files. Tests load the archived
firmware directly, without replacing viewer assets. `audit.py` verifies restoration,
the unchanged gain header, and published firmware identity. The research patch
includes logging and is not a deployment patch.

Every case initializes upright, launches normally, enters CRUISE, engages the
selected controller and settles at 60 m. Normal full-stick input requests a turn
for 50 s, reverses directly for 50 s, releases for 40 s, turns again for 40 s and
releases for 50 s. Two nominal cases start in opposite directions. Six seeded
screen cases vary aerodynamic/suspension damping and brake effectiveness ±30%,
brake slew 0.7–1.1×, wind 0–2 m/s and turbulence 0.1–0.6 m/s. Wind is lateral to
the initial heading. Because subsequent paths differ, turbulent higher-rate
flights are not identical pointwise disturbance histories.

`score.py` was written before flight results. Nominal and perturbed cases use the
same gates: steady mean rate error ≤max(2 deg/s,10% demand), p90 error
≤max(3 deg/s,20%), rate standard deviation ≤max(3 deg/s,15%), altitude RMS ≤3 m
and peak ≤8 m, relative pitch-rate peak ≤30 deg/s and late RMS ≤5 deg/s, true
airspeed fifth percentile ≥3 m/s and minimum ≥2 m/s, no full brake for >5 s.
Final recovery windows require heading-rate RMS ≤3 deg/s, altitude RMS ≤1.5 m,
and relative pitch-rate RMS ≤3 deg/s. Runtime stops on nonfinite physics, loss
of controller ownership, altitude <15 m, roll >60° or pitch >45°.

These are engineering screening thresholds, not certified limits. In particular,
the absolute RMS gates include forced response to turbulence; exceeding them
does not establish growing or self-excited oscillation.

## Sequential decisions

1. `protocol.json`: broad screen at 10 deg/s before automatic expansion. Nominal
   cases passed, but disturbed cases failed absolute motion/tracking thresholds.
   Broad qualification stopped; the pre-generated held-out cohort was not used.
2. `nominal-protocol.json`: separately explore calm-air capacity, without relaxing
   qualification. The 15 deg/s cases remained bounded but failed right-turn
   tracking, so this progression stopped.
3. `boundary-protocol.json`: refine below that failure at 12, 13, 14 deg/s, stopping
   at the first failure. Also compare the classical controller at 15 and 25 deg/s.
4. `diagnostic-protocol.json`: because the 15 deg/s failure was tracking bias with
   good motion/altitude margins, inspect 20 and 25 deg/s to characterize it.
   Proceed beyond 25 only if all original gates pass at 25. These diagnostic
   runs do not undo the failed broad qualification or authorize deployment.

Two startup integration errors (ES module packaging) occurred before any physics
samples. Their empty results are retained under `startup-errors/` and excluded
from flight statistics. The initial compilation error was a missing stdio include;
the source/runtime restoration completed successfully before the corrected build.

## Reproduction

From the microAGU root, with the existing research Python/Node dependencies:

```sh
python research/archive/lqi-turn-envelope-2026-10-17/build.py
python research/archive/lqi-turn-envelope-2026-10-17/run.py
python research/archive/lqi-turn-envelope-2026-10-17/nominal.py
python research/archive/lqi-turn-envelope-2026-10-17/boundary.py
python research/archive/lqi-turn-envelope-2026-10-17/diagnostic_sweep.py
python research/archive/lqi-turn-envelope-2026-10-17/report.py
python research/archive/lqi-turn-envelope-2026-10-17/diagnose.py
python research/archive/lqi-turn-envelope-2026-10-17/audit.py
```

Existing case results are reused rather than overwritten. Use a separate study
directory for a new independent campaign. `RESULTS.md` and `results.json` contain
case-level results. `allocation-diagnosis.json` and `allocation-coupling.png`
explain the constrained-integrator behavior. No public runtime change is made.
