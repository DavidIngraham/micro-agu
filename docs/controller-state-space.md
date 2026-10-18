# Coordinated control: from separate loops to LQI

Ask a powered paraglider to climb and the motor does more than add height. Its thrust acts on the suspended payload, while the aerodynamic forces act on the canopy. Pull a brake to turn and the wing also changes its drag and flight path. The controls we use to steer, climb and regulate speed all act on the same coupled aircraft.

The preceding chapters built separate controllers for turning and longitudinal motion. Here we bring those actions together. The goal is to follow the commanded path while damping the motion between canopy and payload, using throttle and both brakes in a coordinated way.

We will first build the intuition behind that controller, then explain how it estimates the motion it cannot measure, how its gains are designed, and how the complete system is tested. At the end, you can fly the same controller in the browser simulator.

## One input, several responses

The aircraft has three useful control directions: throttle, common brake and differential brake. Common brake means pulling both sides; differential brake creates an imbalance between them. Each affects several aspects of the motion:

- Throttle changes available energy and the forces acting on the payload.
- Common brake changes aerodynamic forces, speed and longitudinal motion.
- Differential brake changes turning motion, but also drag and the flight path.

A separate altitude controller can therefore disturb the motion that a pitch damper is trying to suppress. A turn command can change the power needed to maintain height. Separate loops can work well, especially when their interactions are weak or their response speeds are well separated. Here, those interactions are a central part of the problem.

Select an actuator in the following applet. A small pulse produces several responses, rather than a change in just one variable. This uses the controller's local linear model to isolate the coupling; it is not a nonlinear flight test. Each trace has its own labeled scale.

[Explore one input and several responses](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=coupling)

Coordinated control gives us a way to account for those effects together. It does not remove the coupling; it uses a model of that coupling when choosing the commands.

## Control the motion, not just the error

Consider the payload and canopy at a particular relative pitch angle. If they are moving farther apart, we want to arrest that motion. If they are already returning toward alignment, the appropriate correction is different. The angle alone cannot tell us which situation we are in.

A **state** collects the quantities needed to describe the aircraft's current motion. Our control model includes payload attitude, canopy-relative orientation, air-relative velocity, payload and canopy angular rates, rotor speed and height. Rates matter because they tell us where the motion is heading.

[Compare the same angle with opposite motion](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=motion)

**Full-state feedback** combines these quantities to choose all three control commands together. “Full” refers to the state of the chosen model; it does not imply that the controller knows every wrinkle in the canopy or every gust around it. In this implementation, many of those states are estimates.

The example above uses the actual feedback gains, but prescribes a simple state for teaching. It shows contributions to a requested command, before actuator limits. It does not predict a complete maneuver from those two quantities alone.

## What LQR does, and why we add the I

We need a systematic way to choose how strongly every state should affect each actuator. A **linear quadratic regulator**, or LQR, starts with a linear model near an equilibrium and a set of costs. Those costs express how much we dislike deviations in the state and how much we dislike using control effort. Solving the design problem gives a matrix of feedback gains.

The equilibrium, usually called **trim**, matters. The controls needed to sustain a turn differ from those needed for straight flight. Feedback supplies corrections around the reference state and its associated feedforward controls.

Pure state feedback does not necessarily eliminate a persistent tracking offset when the model and aircraft differ. Integral action supplies a memory of that error. A small height error that persists can build a correction instead of being accepted indefinitely. **LQI** applies the LQR design to an augmented state that also contains accumulated tracking errors.

Our controller accumulates height, turn-rate and airspeed errors. In physical coordinates, the basic structure is:

$$
\mathbf u_{\mathrm{requested}}
=\mathbf u_{\mathrm{feedforward}}
-K_x(\hat{\mathbf x}-\mathbf x_{\mathrm{reference}})
-K_i\boldsymbol\eta.
$$

Here, the estimated state is $\hat{\mathbf x}$, and $\boldsymbol\eta$ holds the accumulated errors. The first feedback term reacts to the estimated motion; the second reacts to errors that have persisted. Feedforward supplies the expected equilibrium action. The firmware normalizes these quantities before applying its stored gains.

This is a tradeoff, not a command to minimize every error at any cost. Stronger correction may demand more actuator motion or energy. Persistent common brake can be expensive, but short brake action can also be useful for damping. The design weights and nonlinear tests determine how those competing objectives are balanced.

## Estimating the motion we cannot measure

The avionics ride on the payload. ArduPilot's EKF supplies navigation and attitude estimates, but we do not have a sensor directly measuring the canopy's relative orientation and angular motion. Those states still affect how the aircraft responds. Feeding back only the directly available quantities would discard information the coordinated controller was designed to use.

The **canopy observer** estimates the missing motion using two steps:

1. **Predict:** use the previous state estimate, the model and the controls actually applied to predict the next state.
2. **Correct:** compare the model's predicted measurements with the available measurements. Use that residual to correct the state estimate.

For the local, normalized model, this is:

$$
\begin{aligned}
\hat{\mathbf x}_{k+1}^{-}&=A\hat{\mathbf x}_{k}+B\mathbf u_k,\\
\mathbf e_{k+1}&=\mathbf y_{k+1}-C\hat{\mathbf x}_{k+1}^{-},\\
\hat{\mathbf x}_{k+1}&=\hat{\mathbf x}_{k+1}^{-}+L\mathbf e_{k+1}.
\end{aligned}
$$

The minus superscript marks the prediction before correction. The measurement matrix $C$ selects the quantities we can observe; the correction matrix $L$ determines how each residual adjusts the estimated states. The deployed implementation also accounts for equilibrium offsets when turning.

Why can a payload measurement correct an estimate of canopy motion? Because the model couples them. A canopy disturbance changes the forces and motion at the payload. Observing that response over time gives information about the hidden state. It does not make the reconstruction exact: model errors, biased wind estimates and weakly observable motion can all limit it.

[Explore prediction, measurement residual and correction](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=observer)

This demonstration uses the released linear plant and observer, with synthetic measurement noise and an initially unknown relative-pitch displacement. It illustrates the mechanism, not measured observer accuracy. Simulator truth is shown only for the reader and is never supplied to the observer.

The onboard observer uses **fixed gains calculated offline**. Its design uses a covariance-based estimation calculation, but it does not propagate a changing covariance during flight. ArduPilot's EKF remains a separate estimator upstream. The current browser configuration uses synthetic airspeed derived from navigation/wind estimation, with no pitot sensor or canopy-angle truth supplied to the controller.

## The online feedback loop

Every 20 ms, while the custom controller is eligible and active, the observer and controller update. References describe the motion we want; measurements describe what the aircraft appears to be doing. The observer supplies a consistent state estimate, and LQI turns its deviation from the reference into coordinated control requests.

[Follow the online control flow](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=online)

The requested controls are not necessarily achievable. Brakes have finite travel and servos have finite speed. Throttle has a ceiling. The allocation stage applies those constraints, and the observer receives the **applied** controls so that its next prediction reflects what the aircraft actually received.

The integrators also need to know when an actuator cannot deliver more action. Otherwise, error can continue accumulating behind a saturated actuator and produce a large correction after it leaves saturation. **Anti-windup** uses the difference between requested and applied controls to manage that accumulated action. In this controller, the correction is actuator-specific, so a common-brake limit need not prevent useful turn-error integration.

A two-second engagement blend manages the transfer into LQI. Selecting it in the interface does not give it ownership in every flight mode: assisted launch, MANUAL, FBWA and landing retain their designated baseline behavior.

## Designing and tuning the current controller

The onboard calculation is a matrix multiplication and a state update. The more involved work happens offline: constructing the model, choosing the design priorities and testing the resulting controller with all of its practical constraints included.

[Follow the offline design and validation process](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=offline)

**Build the nonlinear aircraft model.** Use the as-built 132 mm riser spacing, mass and geometry, then constrain the aerodynamic and joint parameters with flight data. Treat parameters that the logs cannot identify well as uncertainties to test, rather than as precisely known constants.

**Find equilibrium and linearize.** Express the motion in a heading-independent, 16-state coordinate system. Perturb the nonlinear model around trim to obtain a discrete plant at the 20 ms controller interval. Normalize angles, velocities, rates and controls so that the cost weights have a clear scale. Use steady-turn equilibrium information to support the requested turn envelope.

**Define the available measurements and desired outputs.** The observer uses nine channels: payload roll and pitch, three air-relative velocity components, three payload body rates and height. The feedback design augments the 16 modeled states with three integral errors: height, turn rate and speed. Its output has three components: throttle, common brake and differential brake.

**Choose costs and solve for gains.** The current `responsive_softspeed` design emphasizes pitch-rate motion and height tracking, penalizes common-brake effort, and places a softer cost on speed tracking. The discrete Riccati solution supplies the coordinated gains. The observer's fixed correction gains are designed separately from its measurement model and assumed process/measurement noise.

**Evaluate the complete implementation.** Choose candidate costs using nonlinear SITL training cases. Include the 0.7 s climb-reference filter, command limits, trim handling, observer, actuator travel and slew, anti-windup, voltage compensation and engagement behavior. These features change the closed-loop response and must be present when judging a tune.

**Freeze and validate.** Freeze the selected controller before evaluating held-out weather and parameter variations. Assess tracking, internal motion, recovery after disturbances, actuator saturation and energy. A low aggregate tracking cost cannot override a failed recovery screen. If the design changes in response to a validation result, it needs fresh validation rather than continuing to call the same cases held out.

**Export the tested configuration.** Keep the gains, reference parameters, source revision, physics configuration and runtime hashes together. The released command bounds are 35 degrees/s turn rate, 0.75 m/s climb and 1.0 m/s descent. The 3.8–4.4 m/s airspeed target clamp is a bound on the requested target, not a guarantee on actual speed.

<details>
<summary>Design weights and reproducibility</summary>

The selected normalized design adds output costs of 4 for height, 1 for turn rate and 0.3 for speed. Integral-state costs are 12, 3 and 0.3 respectively. A base cost of 0.02 applies to each modeled state, with an additional 90 on each of the payload and canopy pitch-rate states. Input costs are 500 for throttle, 3000 for common brake and 1000 for differential brake. These numbers only have meaning together with the model's normalization and sampling interval.

The [teaching-data generator](../viewer/apps/applets/lqi-explainer/generate.py) reconstructs the selected feedback matrix from the pinned release plant and these costs, and checks agreement with the deployed gains. It also regenerates the local demonstrations and extracts the recorded comparison from the archived campaign data. Its [README](../viewer/apps/applets/lqi-explainer/README.md) specifies assumptions and provenance.

The [runtime manifest](../viewer/assets/runtime/build.json) identifies the actual binary and source hashes. The [longitudinal reproduction instructions](../research/archive/lqi-longitudinal-2026-10-17/REPRODUCE.md) describe rebuilding and rerunning the final campaign. These reproduce the current artifact and its evaluation; obtaining a suitable model for a different aircraft still requires calibration, linearization and new validation.

</details>

## Reading the gains

A gain matrix can look like a wall of unrelated numbers. Read it as a set of contributions: each state deviation affects each requested actuator command, and the controller adds those contributions together.

The heatmap below uses the deployed gains. Its colors show command changes for **labeled example deviations**, rather than raw coefficients whose units differ. Teal increases a command; orange decreases it. Select a cell and vary its deviation to see how the same state contributes to all three outputs. The feedback minus sign is already included.

[Explore the gain matrix and command contributions](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=gains)

These example deviations are teaching scales, not measured distributions of flight states. A large cell does not, by itself, prove a state is especially valuable: its typical magnitude, estimation quality, correlation with other states and actuator saturation also matter. The current controller combines all of them at once.

## What the complete system achieves

Once the mechanism is clear, the useful comparison asks three things together: does it follow the command, does the internal motion settle, and what effort does that require?

This recording shows a matched case from the longitudinal validation. Switch between LQI and the classical PG + TECS controller, and inspect altitude error, relative pitch rate and throttle through the test phases. The plotted recording is downsampled for presentation; the full-resolution campaign metrics remain the authority for peak values.

[Inspect a matched nonlinear SITL recording](https://davidingraham.github.io/assets/microagu-viewer/lqi-explainer.html?view=replay)

Across eight matched disturbance cases, the median LQI / PG + TECS ratio was 0.653 for relative pitch-rate RMS, 0.529 for its peak and 0.956 for modeled shaft energy. These results support a substantial damping improvement and a smaller energy improvement for these cases. They do not show that every flight task benefits equally.

<details>
<summary>Per-case evidence and actuator limits</summary>

[Explore brake authority and the release evidence](https://davidingraham.github.io/assets/microagu-viewer/lqi-development.html)

The brake diamond is a simplified geometric illustration using the mean and half-difference of left/right travel. Those coordinates are not the firmware's common/differential allocation convention. The actual allocator also includes slew limits and calibration.

All fourteen LQI validation flights passed their frozen screens. A classical comparator exceeded its quiet pitch-rate RMS screen; that failure remains in the [full validation record](../viewer/assets/runtime/lqi-longitudinal-validation.json). The tests include reduced voltage, turns, Dryden turbulence, thermals, coefficient uncertainty, actuator variation and controller transitions. Modeled shaft energy excludes electrical and servo losses.

</details>

These are finite simulator tests, not flight qualification or a statistical reliability estimate. Their usefulness depends on the model representing the motions and disturbances that matter. Flight evidence remains necessary.

## Fly the controller in your browser

The explanatory applets above either illustrate the local model or replay recorded results. The **browser flight simulator** runs a live aircraft instead: the Rust nonlinear model supplies motion and simulated sensors to ArduPilot SITL, with its EKF and controllers in the loop. Your inputs and the weather controls change the flight as it runs.

[Open the interactive flight simulator](https://davidingraham.github.io/assets/microagu-viewer/simulator.html)

LQI is selected by default; PG + TECS is available in the controller selector. Allow the launch sequence to finish and the aircraft to settle in an eligible autopilot mode. Observe altitude/path tracking and canopy–payload motion separately, then try a modest turn or altitude change. Repeat with the other controller after resetting and settling the aircraft.

Weather and trajectories are not necessarily synchronized between interactive runs. Use the recorded matched tests for quantitative comparisons, and the live simulator to build a feel for the coupled motion. The [simulator guide](../viewer/SIMULATOR.md) describes flight modes, touch controls, weather and plots.

<details>
<summary>Earlier analytical reference</summary>

The [frequency-response and state-space study](controller-state-space-archive.md) is retained as a technical archive. It describes an earlier controller and sensing configuration, rather than the implementation introduced here.

</details>
