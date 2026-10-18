# Developing the LQI controller

A paraglider can follow an altitude command and still rock badly beneath its canopy. Our controller work started with that distinction: reaching the requested flight path and damping the aircraft's internal motion are related, but they are not the same task.

The classical controller remains a useful baseline. It couples a paraglider turn controller to ArduPlane's TECS longitudinal control. The LQI development asks whether one coordinated controller can use throttle and both brakes to track the path while managing the motion between canopy and payload.

This page follows that development through simulation. The browser currently selects the experimental LQI by default, with PG + TECS available for comparison. The results below are finite simulator tests, not flight qualification. The [earlier frequency-response and state-space study](controller-state-space-archive.md) remains available as a technical archive; its controller and sensing assumptions differ from today's implementation.

## One aircraft, several competing objectives

A turn needs differential brake. Symmetric brake can help damp longitudinal motion, but sustained braking can also increase drag. Throttle changes flight-path energy and can excite the suspended payload. Treating those actions independently makes it harder to account for their combined effects.

The useful question therefore became broader than “which controller has the smallest altitude error?” We want accurate tracking, motion that settles after disturbances, acceptable actuator demand, and reasonable energy use. An improvement in one does not automatically compensate for a failure in another.

Early matched simulation tests exposed persistent pitch oscillations in some classical-controller CRUISE cases after the weather disturbance ended. LQI settled much more strongly in those cases, but early AUTO results also showed worse altitude and path tracking. That was a reason to keep developing and testing, rather than declare a winner from the damping plots alone.

## What LQI adds

The controller works around a modeled equilibrium. A local linear model describes how small changes in aircraft state and control affect the subsequent motion. Feedback uses the estimated state deviation to choose coordinated changes in throttle, symmetric brake and differential brake.

An LQR penalizes state deviations and control effort. LQI augments that problem with accumulated tracking errors, so persistent offsets can generate a correcting action even when the immediate error is small. In schematic form, with the error sign defined here as measured output minus reference:

$$
\begin{aligned}
\dot{\boldsymbol\eta} &= \mathbf y-\mathbf r,\\
\delta\mathbf u &= -K_x\,\delta\hat{\mathbf x}-K_i\boldsymbol\eta,\\
\mathbf u_{\rm requested} &= \mathbf u_{\rm trim}+\delta\mathbf u.
\end{aligned}
$$

The design balances these terms through a quadratic cost on the augmented state and control effort. The gains come from the model and chosen weights; those weights still need to be selected against the nonlinear aircraft and real actuator constraints.

| Stage | Information entering it | What it contributes |
|---|---|---|
| References and trim | Flight-mode commands, turn and climb demand | Feasible commands and an equilibrium about which feedback acts |
| EKF and canopy observer | Navigation and attitude estimates, available measurements and applied controls | Estimated motion, including canopy states that are not directly measured |
| State feedback | Estimated deviations from equilibrium | Coordinated damping and tracking action |
| Integral feedback | Accumulated tracking errors | Rejection of persistent offsets |
| Allocation and anti-windup | Requested controls, travel and slew constraints | Achievable actuator commands and feedback about unavailable authority |
| Aircraft and sensors | Applied throttle and brakes | The next motion and measurements, closing the loop |

The current browser controller retains the EKF and uses synthetic airspeed rather than a pitot sensor. Its canopy observer has fixed gains; it does not dynamically propagate uncertainty like a full Kalman filter. It receives no canopy-angle truth from the simulator. Extra sensing was investigated separately, and should not be credited to the deployed results.

## The actuator limits are part of the controller

An unconstrained linear controller can ask for a brake to move beyond its stop or faster than its servo permits. It can also accumulate integral action while that request is impossible. The eventual response can then be very different from the linear design.

The diagram below makes the shared brake budget visible. For this normalized illustration, common brake is the mean of left and right brake, and differential brake is half their difference:

$$
 b_L=c+d,\qquad b_R=c-d,\qquad
 |d|\leq c\leq 1-|d|.
$$

Near zero common brake, a large differential request would require one brake to push beyond its fully released position. At the other extreme, applying both brakes fully leaves no differential authority. The firmware includes additional allocation and slew behavior; the diagram isolates the travel geometry so this coupling is easy to see.

[Explore brake authority and the release evidence](https://davidingraham.github.io/assets/microagu-viewer/lqi-development.html)

This is why the practical development extended beyond a gain matrix. Applied actuator feedback, actuator-specific anti-windup, reference filtering and trim handling all affect whether the controller can deliver the action it was designed to produce.

## How the design became a usable controller

**Start with a model, then challenge it.** Linearization and state-space analysis supplied a way to reason about coupled motion and choose candidate gains. The archived study documents those early diagnostics. A favorable linear result was only a starting point for nonlinear, autopilot-in-the-loop tests.

**Judge recovery as well as error.** Disturbance tests examine what happens after the disturbance stops. Persistent oscillation, peak motion, quiet-period residuals and actuator saturation matter alongside RMS tracking error. A favorable weighted score cannot override a failed stability or recovery screen.

**Retune with the actual command shaping in place.** The selected `responsive_softspeed` tune was evaluated with the 0.7 s climb-reference filter included. In twelve fresh validation cases its composite score improved in every case against the published comparator, with a median reduction of about 41%. Energy did not improve uniformly: the median paired change was about +0.3%, with improvements in five of twelve cases. This was primarily a tracking and recovery improvement, not a universal efficiency win. The score is specific to that campaign and is not interchangeable with the later release metrics.

**Handle equilibrium and transitions.** Expanding turn demand required steady-turn trim handling and observer equilibrium recentering. A two-second engagement blend softened the transfer from the previously applied commands into LQI. Tests of an already settled controller would have missed that handover problem.

**Leave authority for disturbances.** Nominal 1 m/s climb tests passed, but used nearly all of the nominal 60% throttle ceiling. The released climb command was limited to 0.75 m/s instead. During reduced-voltage validation, a mismatch between voltage-compensated motor feedback and uncompensated controller requests also produced false anti-windup feedback. Correcting those units was necessary before accepting the expanded envelope.

The [turn-release notes](../viewer/LQI-FEASIBLE-TURNS.md) and [longitudinal-release notes](../viewer/LQI-LONGITUDINAL.md) describe the integration changes, retained failures and validation limits. The [filter-retuning report](../research/lqi-filter-retune-2026-10-17/RESULTS.md) records the earlier selection and its mixed energy results.

## What the current comparison supports

The interactive chart uses the eight matched disturbance comparisons from the longitudinal release. Across those cases, the median LQI / PG + TECS ratio was 0.653 for relative pitch-rate RMS, 0.529 for its peak, and 0.956 for modeled shaft energy. The largest ratio in each category was 0.694, 0.787 and 0.978 respectively. Lower is better for each of these metrics.

That supports a substantial damping improvement and a smaller modeled energy improvement for these cases. It does not establish a field reliability rate or show that LQI wins under every flight task. Energy here excludes electrical losses and servo consumption.

The full validation also included absolute tracking and recovery screens. All fourteen LQI validation flights passed their frozen gates. One classical comparator exceeded the quiet pitch-rate RMS screen; that failure remains in the record rather than being hidden by the paired ratios. Tests covered turn demand, reduced voltage, Dryden turbulence, thermals, coefficient uncertainty, actuator slew variation and mode/controller transitions.

[Read the machine-readable protocol and all retained outcomes](../viewer/assets/runtime/lqi-longitudinal-validation.json). The record includes source and binary hashes so these results can be associated with the tested configuration.

## What is running in the browser

| Item | Current published configuration |
|---|---|
| Physics geometry | As-built 132 mm riser spacing |
| Controller | Experimental LQI selected by default; PG + TECS selectable |
| Sensing | EKF synthetic airspeed and internal canopy observer; no pitot or canopy truth |
| Turn-reference limit | 35 degrees/s |
| Vertical command limits | 0.75 m/s climb; 1.0 m/s descent |
| Climb-reference filter | 0.7 s |
| Airspeed target clamp | 3.8–4.4 m/s; not a guarantee on actual airspeed |

Selecting LQI does not mean it owns the actuators in every mode. Assisted launch, MANUAL, FBWA and landing retain their designated baseline behavior; the custom controller takes over only in eligible conditions. The [simulator documentation](../viewer/SIMULATOR.md) describes the current controls and mode behavior.

[Fly the browser simulator](https://davidingraham.github.io/assets/microagu-viewer/simulator.html).

The next useful improvements must survive the same distinction that started this work: following the route is necessary, but the aircraft also needs to settle, preserve control authority and use power sensibly. Broader validation and flight evidence are needed before treating the simulated envelope as an operational one.
