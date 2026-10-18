# Filtered common-brake penalty

AI-assisted implementation and simulation study. No flight qualification or deployment.

Keep the existing flight-log calibration, 132 mm riser spacing, zero common trim,
instantaneous input cost 1000 and pitch-rate weight 30. Add one controller state:
`z[k+1] = exp(-0.02/3) z[k] + (1-exp(-0.02/3)) applied_common/0.1`.
The observer remains 16 states; integrators remain three. The new state is updated
from allocated input, initialized at engagement, and included in handover matching.
It is not an extra sensor or a physics truth input.

First replay cases 0,1,3,5,6,9,13,15 with zero memory cost and diagnostics.
Then screen memory costs 1000 and 3000, retaining all failed runs. Cases include
AUTO tracking, the original and rerun PG+TECS oscillation cases, and persistent brake.
Use the existing 20–100 s disturbed / 100–180 s recovery protocol.

Acceptance before energy selection: every run completes, CRUISE late pitch AC RMS
below 0.2 degrees, and mode median disturbed altitude/cross-track RMS no more than
25% above the diagnostic zero-cost control. Inspect individual outliers as well.
Expand a competitive candidate to all 16 LQI cases. This reuses tuning scenarios;
it is a sensitivity/regression study, not an independent statistical validation.
Compare matched windows and use the same quasi-steady shaft-power estimate as the
archived campaign. Brake travel is not a percentage energy loss.

SITL-only diagnostic CSV fields: AP time_ms; four normalized references
(height, heading rate, speed deviation, climb); measured TAS; speed target;
nine normalized measurements; 16 normalized observer states; three integrators;
requested throttle/common/differential; applied throttle/common/differential;
filtered common brake normalized by 0.1. Harness timestamps each record against
simulation time. Sampling is 5 Hz. Diagnostics compile out of production builds.

`trial.sh NAME WEIGHT [CASE_IDS]` saves source/gain/firmware snapshots, builds real
WASM SITL and runs the unchanged scenarios. `analyze.py NAME` reconstructs the
late-window common-brake command from reference, state, integral and memory terms.
The short startup matching term has decayed by that window.
