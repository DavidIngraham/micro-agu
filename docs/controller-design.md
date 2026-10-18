# Designing the Micro-AGU Flight Controller

*Micro-AGU · Controller design, part 2 · [Through Trout Lake](first-flight-controller.md) · [Build](micro-agu-design.md) · [Flight tests](flight-testing.md) · [Controller design](controller-design.md) · [Data and methods](data-and-methods.md)*

The controller has two jobs: turn the aircraft along its route and use throttle to follow the requested altitude. Both jobs act on the same suspended system. A brake correction can excite payload pitch, and a throttle correction changes the motion seen by the steering loop. The design uses feedforward for the expected effort, modest feedback for tracking, and separate damping paths shaped around the coupled modes.

The follow-up [state-space controller study](controller-state-space.md) compares joint tuning of these loops with actuator/sensor diagnostics and a full-state feedback benchmark, using the same sampled linear plant and matched nonlinear tests.

[Part 1](first-flight-controller.md) follows my independent preflight design work through the Trout Lake missions: adapting the roll loop, changing TECS to throttle-only altitude control, and disabling a pitch damper that made the motion worse. This second phase uses those flights to inform the articulated model and design the replacement controller. It brings steering and altitude together around the resulting architecture, its classical analysis and its simulation evidence.

## From flight logs to a model for controller design

The simple simulator had been useful for choosing the gains I took to Trout Lake. After the flights, I wanted it to answer a harder question: would a controller that looked good in simulation behave sensibly on the actual suspended aircraft? The achievable turn radius looked optimistic, sustained bank was harder to maintain in flight, and the pitch damper had exposed motion that a single rigid aircraft could not explain.

The starting evidence was the Hood River and Trout Lake logs: actuator commands, logged parameter ranges, payload attitude and rates, navigation velocity, altitude and battery measurements. Hood River included substantial manual flying to feel out the platform. Early Trout Lake throttle tests and the later sustained turns supplied more ordinary operating points. The powered, high-bank Hood River turn was useful as a stress case, but reproducing that maneuver stopped being the main objective. The controller needed a useful model of everyday glide, climb and sustained turns first.

### Reconstruct the aircraft before fitting its response

The two flights were not the same aircraft configuration. All-up mass was 1.385 kg without the added ballast and 1.758 kg with it—a 373 g difference, carried at the bottom of the payload. I had also adjusted the rigging. We therefore kept separate flight configurations instead of asking one set of coefficients to explain both through mass alone.

The logged PWM ranges mattered just as much as mass. Both main flights used 800–2100 μs on the left brake and a reversed 900–2400 μs range on the right, with no recorded in-flight changes to those ranges. The nominal conversion using the 55 mm drums and servo mapping gives about 62.4 mm of configured left travel and 72.0 mm on the right. The asymmetry is real; treating both outputs as identical normalized brakes would fit the wrong inputs. Effective slack and preload are separate from physical drum travel. Their fitted values can absorb rigging and canopy asymmetry, so they are not measurements of line length.

![Flight-derived throttle and climb relationships for the two configurations](assets/flight-testing/throttle-climb-fit.png)

*Steady flight windows establish useful throttle/climb relationships. They do not identify the complete transient dynamics. Hood River’s erroneous current readings and false power limiting also prevent treating that flight as a clean propulsion calibration; the missing input capacitor was fixed before Trout Lake.*

The propulsion model uses the reported FFTY 3115-900KV motor and 9×4.5 propeller geometry, with an APC 9×4.5 aerodynamic map as a proxy. It includes rotor speed dynamics, reaction torque, rotor angular momentum and precession, and the reaction to spin acceleration. Rotation is clockwise viewed from behind. Component weights and CAD-informed dimensions supplied approximate mass distribution and inertia; these were engineering estimates where direct measurements were unavailable.

### Let the payload and canopy move independently

The important structural change was moving to two rigid bodies connected by a point joint. Each body has its own attitude and angular velocity. Their shared attachment constrains relative translation while allowing relative roll, pitch and yaw: twelve free-body degrees of freedom minus three positional constraints gives **nine mechanical degrees of freedom**.

The purpose of this change is to separate **what the canopy does** from **what the payload-mounted sensors see**. The canopy generates most of the aerodynamic support and steering forces. The payload carries most of the mass, the motor and the IMU. A throttle or brake change acts on one part of that system first; the suspension transfers forces and moments to the other. Treating them as one rigid aircraft would remove that relative motion before the controller ever saw it.

![Modeled canopy and payload, with separate free-body diagrams showing aerodynamic loads, weights, thrust and equal-and-opposite joint reactions](assets/model-structure/modeled-system-fbd.svg)

*Left: the two rigid bodies and their equivalent attachment. Right: the loads on each isolated body. Orange arrows are external loads; green arrows are internal joint reactions. Directions are illustrative, not a solved trim condition. The suspension sketch is not a set of individually simulated lines, and the force arrows are not to scale.*

### What is connected to what?

The joint is an **equivalent point connection**, not a pin that permits only one rotational axis. If $\mathbf{x}_p$ and $\mathbf{x}_c$ are the payload and canopy CG positions, $R_p$ and $R_c$ their attitudes, and $\mathbf{a}_p$ and $\mathbf{a}_c$ the fixed attachment offsets in their respective body frames, the shared-point condition is

$$
\mathbf{x}_p + R_p\mathbf{a}_p
= \mathbf{x}_c + R_c\mathbf{a}_c.
$$

The equations solve for a reaction force $\mathbf{J}$ that makes the two attachment-point accelerations agree, including the rotational acceleration and centripetal terms. The payload receives $+\mathbf{J}$ and the canopy receives $-\mathbf{J}$, each acting through its own attachment-to-CG moment arm. There are three independent relative rotations, but they are not torque-free: an effective spring and damper oppose relative attitude and angular rate. That torque also enters the two bodies with opposite signs. The coefficients represent the suspension's aggregate behavior; they do not model fabric deformation or individual line tensions.

The force model has four main pieces:

| Component | What the simulation calculates | Why it matters to the controller |
|---|---|---|
| Canopy aerodynamics | Lift and drag on five panels across a curved **rigid** wing, using local airflow including rotational velocity; left/right brake pull changes those loads. Panel moment arms and additional aerodynamic moment terms produce roll, pitch and yaw moments. | A brake command can change support, drag and attitude together; it is not a direct turn-rate command. |
| Payload loads | Gravity, air-relative drag, and thrust applied at the motor's offset from the payload CG. | A thrust increase can immediately rotate the payload before changing the aircraft's climb rate. |
| Propeller dynamics | Voltage and throttle set a target RPM with a first-order lag. RPM and axial airflow determine thrust and shaft torque through the propeller map. Rotor angular momentum adds gyroscopic and spin-acceleration reactions. | Throttle response has dynamics, and payload rotation can couple pitch and yaw. |
| Inertia and suspension | Separate body inertias, canopy added-air inertia, the point-joint reaction, and rotational restoring/damping torques. | These determine how the two bodies exchange motion and which oscillations appear in the payload gyro. |

Added-air inertia represents the surrounding air that must accelerate with the canopy. It enters the canopy's coupled translational/rotational mass matrix; it is neither another rigid body nor an extra lift arrow. Likewise, the five panels are aerodynamic integration elements on one canopy, not five articulated wing segments.

### How the five-panel canopy produces loads

The canopy is one rigid body, divided into **five equal-area aerodynamic integration regions** along its curved span. The panel centers have different positions and local orientations. Lift and drag are evaluated at each center, then combined into the force and moment on the canopy.

[Explore the five-panel canopy aerodynamics](https://davidingraham.github.io/assets/microagu-viewer/canopy-panels.html)

*Select symmetric flight or right braking, inspect any panel, and switch between three-quarter and front views. These are frozen operating points calculated by the current Rust plant with the as-built Trout Lake SITL preset: 7 m/s, 4.58° canopy angle of attack, zero sideslip and zero angular rate. They illustrate the force calculation; neither case is claimed to be a trimmed equilibrium or a predicted turn trajectory. The earlier refinement results below retain their historical model provenance.*

For panel $i$, its velocity through the air in local panel coordinates is

$$
\mathbf{v}_i = R_i^T\left[\mathbf{v}_{c,\mathrm{air}} + \boldsymbol{\omega}_c\times\mathbf{r}_i\right].
$$

Here $\mathbf{r}_i$ is the panel center's offset from the canopy CG, $R_i$ maps the panel frame to the canopy frame, and $\mathbf{v}_{c,\mathrm{air}}$ and $\boldsymbol{\omega}_c$ are expressed in canopy coordinates. The blue arrows show incoming airflow, opposite this local velocity. Even in a uniform wind, canopy rotation changes the velocity at the different panel centers.

The two left panels receive left brake pull; the two right panels receive right brake pull; the center panel receives their average. Local angle of attack and brake pull change lift and drag coefficients. The model also applies an effective aerodynamic inclination change proportional to differential brake pull. This is a modeled aerodynamic correction, not five physical hinges or simulated fabric deformation. The front inset shows those effective inclinations; the main surface retains its rigid geometric arc.

The panel forces and their moment arms contribute

$$
\mathbf{F}_{\mathrm{panels}}=\sum_{i=1}^{5}(\mathbf{L}_i+\mathbf{D}_i),
\qquad
\mathbf{M}_{\mathrm{panels}}=\sum_{i=1}^{5}\mathbf{r}_i\times(\mathbf{L}_i+\mathbf{D}_i).
$$

The plant then adds separate whole-canopy side-force and aerodynamic moment terms, based on sideslip, angle of attack, angular rate and differential braking. The figure lists the panel moments and additional aerodynamic moments separately, so the distributed-panel calculation is not mistaken for the entire aerodynamic model. Gravity, suspension reactions and added-air inertia enter elsewhere in the body equations.

### How the equations become a simulation step

![Simulation block diagram from commands and actuator mapping through canopy and payload loads, constrained acceleration solving, integration and payload sensor feedback](assets/model-structure/simulation-blocks.svg)

*The same mechanics support recorded-input replay, standalone controller experiments and native ArduPilot SITL. The outer feedback path changes between those uses; the two-body force and constraint calculation remains the plant.*

Each step converts commands into throttle and effective left/right brake pull, with the selected actuator mapping and rate limits. The force routines use the current body motion, wind and rotor speed. The coupled solver combines those loads with the inertia matrices and joint constraint to obtain both bodies' accelerations. RK4 substeps advance their motion, and quaternion normalization maintains valid attitudes. Updated motion then changes the next aerodynamic and joint loads: this is a nonlinear feedback process inside the plant, even when replaying fixed actuator commands.

The implementation stores **26 body-state values**: position (3), attitude quaternion (4), translational velocity (3) and angular velocity (3) for each body. Those are redundant coordinates, not 26 independent degrees of freedom. Unit-quaternion conditions and the shared attachment's position/velocity constraints leave the nine mechanical freedoms and their velocities. Rotor RPM is retained separately, and the actuator wrapper retains brake position. Controller integrators and filters belong to the controller, rather than to these mechanical coordinates.

Finally, the sensor interface samples the **payload**, including the IMU's offset from its CG. Standalone analysis can use ideal observations; native SITL constructs payload-frame gyro and specific-force measurements plus navigation quantities for ArduPilot's estimator. Full canopy attitude is available to analysis and rendering, but is not silently supplied as a measured flight-controller input. Wind and voltage are inputs to this model; the constant-voltage mission runs below do not contain a battery-discharge model.

These details describe the [mechanics](https://github.com/DavidIngraham/ardupilot/blob/5f483ade60/Tools/simulation/paraglider/dynamics.cpp), [simulation wrapper](https://github.com/DavidIngraham/ardupilot/blob/5f483ade60/Tools/simulation/paraglider/simulator.py) and [SITL sensor interface](https://github.com/DavidIngraham/ardupilot/blob/5f483ade60/Tools/simulation/paraglider/json_backend.py) used by this refinement study. The diagrams are explanatory views of that model, not additional calibration evidence.

### See the separate motions

[Open the interactive 3D motion tour](https://davidingraham.github.io/assets/microagu-viewer/joint-tour.html) to step through the three translations, three payload rotations and three relative canopy rotations. Use front, side or top views, pause, scrub, or slow playback. The default disturbance view releases either the canopy or payload, with a 3°–15° amplitude slider and both stiffness profiles. Select **Closed loop · AUTO** at 9° to see the native autopilot respond after flying a straight waypoint leg at 100 m altitude for 100 seconds. **Open loop · fixed trim** shows the original natural response with throttle and brakes held at equilibrium trim. Throttle, brake pull, altitude deviation and ground-track deviation appear below the motion.

[Explore the joint motions](https://davidingraham.github.io/assets/microagu-viewer/joint-tour.html)

*The first nine segments prescribe one coordinate at a time to explain the degrees of freedom. The final three replay simulated roll, pitch and yaw deflection releases with the mission profile. Prescribed motion is labeled separately from physics. The response view removes shared translation, preserving all rotations and relative displacement at their simulated amplitudes. The canopy is schematic; the carriage uses the SolidWorks model.*

A curved five-panel canopy supplies aerodynamic forces and moments. Effective joint stiffness and damping represent the suspension’s resistance to relative rotation, including a mass-dependent yaw restoring term. Thrust acts at the motor’s location relative to the payload CG. That geometry lets the carriage pitch nose-down immediately after a throttle increase while the aircraft eventually climbs. The simulated IMU follows the payload, including its offset from the CG; it does not report canopy attitude.

This is what makes the articulated animation relevant to control design: steering can excite pitch, and the payload gyro can see a motion quite different from the wing’s. Rotor and actuator dynamics add states to the equations without changing the nine mechanical freedoms.

### Why a pitch disturbance also produces yaw

In the [interactive joint-response viewer](https://davidingraham.github.io/assets/microagu-viewer/joint-tour.html), select **Open loop**, **Payload-only deflection**, **Pitch release**, **Mission profile**, and **9°**. The carriage pitches and also yaws, even though the brake commands stay fixed. Switch to **Canopy-only deflection** to see the same coupling excited through the wing.

The spinning propeller is the main source of that yaw in these simulated cases. Its angular momentum lies along the motor shaft. When the payload pitches, changing the direction of that momentum produces a gyroscopic reaction torque about the yaw axis. The model applies $\boldsymbol{\tau}_{\mathrm{gyro}}=-\boldsymbol{\omega}_{\mathrm{payload}}\times\mathbf{H}_{\mathrm{rotor}}$.

I checked this by replaying both releases with the rotor angular-momentum terms disabled, while retaining thrust, shaft reaction torque, body inertia, initial state, RPM, and fixed trim controls. Over the eight-second release, the maximum absolute change in **payload yaw from its initial heading** was:

| Initial pitch disturbance | Normal model | Rotor momentum disabled |
|---|---:|---:|
| Payload only, +9° | 7.1° | 0.7° |
| Canopy only, +9° | 11.7° | 0.8° |

That large reduction identifies gyroscopic coupling as the dominant source of the visible yaw in these cases. A canopy disturbance also drives payload pitch, which then excites the rotor coupling. The smaller remaining yaw comes from the rest of the coupled model. This establishes the mechanism in the simulator; its magnitude depends on the estimated rotor inertia and has not been isolated in flight data. The intervention removes both gyroscopic and spin-acceleration reaction terms, rather than switching off propeller thrust or shaft torque. [Numerical results](assets/viewer/joint-tour/pitch-yaw-coupling.json) and the [reproduction script](https://github.com/DavidIngraham/micro-agu/blob/main/research/viewer-studies/check_pitch_yaw_coupling.py) record the comparison.

<a id="watching-the-coupled-aircraft-move"></a>

### Fit useful behavior, retain competing explanations

We compared recorded-input responses over ordinary flight windows, examined steady glide/climb and turn equilibria, and checked attitude-rate spectra as well as trajectory error. A good ground track alone could hide the wrong rocking motion. Conversely, matching a rate RMS did not establish that the right physical mode produced it. Trim solving became part of the process so a controller comparison did not begin with an arbitrary force or moment imbalance.

Two profiles survived that work. The **mission profile** better represents ordinary trajectory performance and is the baseline used below. The **dynamic-roll profile** uses a softer roll joint with a corresponding trim correction; it better represents the observed faster roll motion. Keeping both exposes a real uncertainty: joint mechanics, damping, rigging and external excitation are not uniquely recoverable from these flights. The longitudinal screening includes that alternative, while the lateral filter study uses the mission profile.

The resulting model is more useful for controller design because it represents asymmetric brake authority, configuration-dependent trim, propulsion dynamics and the coupled motions missing from the original approximation. It is not a unique identification of the aircraft. Cross-axis phase errors and missing low-frequency yaw energy remain, and the powered Hood River equilibrium has a slow instability in the model. The same two flights guided development, so replay agreement is not independent flight validation.

### Connect the model to the real controller

The retained dynamics run through ArduPilot’s JSON SITL backend. Native navigation, TECS, steering code and the estimator receive simulated payload sensor data and return PWM commands through the flight-specific actuator mapping. Tests start from a solved airborne trim. This allows the full mission controller to be evaluated against the same mechanics used in the analysis, rather than relying only on a standalone ideal-sensor controller.

The model assumes an inflated canopy and a taut point connection. It does not simulate inflation, slack lines, collapse or landing. The canopy is rigid, the stall law is approximate, and the mission comparisons use constant voltage rather than battery discharge. Those limits set the scope of this article: up-and-away control, ordinary maneuvers and sensitivity to modeled disturbances.

[Model mechanics and configuration notes](https://github.com/DavidIngraham/ardupilot/blob/5f483ade60/Tools/simulation/paraglider/README.txt) preserve the calibration assumptions, competing profiles and reproduction details. The [data and methods appendix](data-and-methods.md) separates flight evidence from subsequent model and controller tests.

## One aircraft, two control paths

Navigation supplies a turn request and an altitude trajectory. Differential brake pull controls turning; throttle controls climb and descent. Symmetric braking is reserved for future landing work, rather than acting as an elevator during normal flight.

The analysis uses the 1.758 kg Trout Lake configuration, asymmetric brake trim, and modeled 9×4.5 pusher propulsion. Canopy and payload have separate attitudes. Rotor dynamics and active steering remain in the longitudinal plant.

Nine mechanical degrees of freedom do not mean nine linear states. The local plant uses 15 perturbation states after removing absolute position and arbitrary heading, including rotor RPM. Steering filters and integration add three states; altitude, climb integration, pitch filters and reference smoothing add five more to the full longitudinal model.

## Steering: track turn rate, damp roll separately

![Heading-rate controller with separate feedforward, PI correction and roll-rate damping](assets/control-loops/current-lateral.png)

Feedforward supplies the expected differential-brake input. Heading-rate PI corrects tracking and asymmetric trim, while payload roll-rate feedback damps rocking. These paths have different jobs and independently adjustable gains.

Navigation's bank-encoded request is converted into heading rate, limited to ±25°/s, smoothed with a 0.1 s time constant, and limited to 40°/s² acceleration.

~~~text
heading_rate = (q sinφ + r cosφ) / cosθ
e = shaped_target − filtered_heading_rate
Sᵥ = clamp(5 / max(Vair, 2), 0.25, 2.5)

u_brake = clip[Sᵥ(0.009 target + 0.005 e − 0.003 filtered_p) + I, −1, +1]
İ = 0.002 e, with |I| ≤ 0.15 and conditional integration
~~~

Here p is body roll rate, q body pitch rate and r body yaw rate; controller rates are in degrees per second. Body roll rate only approximates the Euler bank-angle derivative near level attitude. The brake mixer maps the normalized output to the actual servo ranges.

The small P gain is deliberate. Large heading-rate feedback excited the payload instead of simply making turns more accurate. Feedforward carries most of the steady demand; feedback corrects the residual.

## Lateral classical analysis: why this steering loop works

The steering design needs enough low-frequency authority to track a turn without feeding the suspended modes. To quantify that tradeoff, I linearized the 9-DOF model at the Trout Lake straight, level trim: 1.758 kg, 4.383 m/s, 44.59% throttle, left brake released and right brake at 3.747 mm. That nonzero brake trim matters: small corrections stay on the right-brake branch rather than crossing the mixer’s change of side.

The local model keeps fourteen internal states after removing global position and heading. That is a state count, not fourteen mechanical degrees of freedom. Throttle and equilibrium rotor speed are held fixed for this lateral analysis. With differential brake command $u$, heading rate $\dot\psi$, and body roll rate $p$, the plant is:

$$
\begin{aligned}
\Delta\dot{x} &= A\Delta x+B\Delta u \\
G_\psi(s) &= \frac{\Delta\dot{\psi}}{\Delta u}=C_\psi(sI-A)^{-1}B \\
G_p(s) &= \frac{\Delta p}{\Delta u}=C_p(sI-A)^{-1}B \\
G_\psi(0) &= 98.00\ \text{deg/s per brake fraction} \\
G_p(0) &= -13.05\ \text{deg/s per brake fraction}
\end{aligned}
$$

A one-percent brake-command increment therefore produces about 0.98 deg/s of steady heading rate locally. Inverting that gain, including the speed multiplier, gives a feedforward gain of **0.00894**—almost exactly the selected 0.009. The linear plant therefore gives a physical basis for the feedforward setting, rather than leaving it as an empirical gain.

![Linear plant Bode plots checked against nonlinear sine tests](../research/flight-log-analysis/figures/turn-model-spike/classical-lateral/plant-bode.png)

*The full plant has several coupled modes. Small-amplitude nonlinear sine tests match the sampled linear response within 0.005%; the slight difference from the continuous curve at high frequency is the input hold. This checks the linearization against the simulator.*

### The plant and closed-loop transfer functions

At fixed speed scaling, below the command and actuator limits, the implemented controller becomes:

$$
\begin{aligned}
e(s) &= r(s)-H(s)\dot{\psi}(s) \\
u(s) &= SK_{\mathrm{FF}}r(s)+\left(SK_P+\frac{K_I}{s}\right)e(s)-SK_{\mathrm{RD}}H(s)p(s) \\
K_{\mathrm{FF}} &= 0.009,\quad K_P=0.005,\quad K_I=0.002,\quad K_{\mathrm{RD}}=0.003 \\
S &= \frac{5}{4.383}=1.1408 \\
H(s) &= \frac{25.133}{s+25.133}\quad\text{(4 Hz measurement filter)} \\
r(s) &= \frac{r_{\mathrm{requested}}(s)}{0.1s+1}\quad\text{(command limits inactive)} \\
L(s) &= H(s)\left[\left(SK_P+\frac{K_I}{s}\right)G_\psi(s)+SK_{\mathrm{RD}}G_p(s)\right] \\
\frac{\dot{\psi}(s)}{r_{\mathrm{requested}}(s)} &= \frac{G_\psi(s)\left[S(K_{\mathrm{FF}}+K_P)+K_I/s\right]}{(1+L(s))(0.1s+1)}
\end{aligned}
$$

The integrator is not speed-scaled. Feedforward affects command response but does not move the feedback poles. Also, body roll rate is not zero in every steady banked turn: $p=\dot\phi-\dot\psi\sin\theta$. Roll-rate damping therefore supplies a small steady-turn correction as well as opposing oscillation.

### Poles, zeros and the feedback-gain limit

The selected gains are locally stable, but the original larger feedback gains are unstable in this model. With the selected gains, a lightly damped pair sits at $(-0.986\pm j15.969)\,\mathrm{s}^{-1}$: about 2.54 Hz, with damping ratio 0.062. Scaling all feedback gains together reaches instability at **1.524 times nominal**, a gain margin of **3.66 dB**. Increasing heading-rate P alone reaches the boundary near 0.00791, with I and roll damping held fixed.

![Root locus annotated with physical mode participation and combined-loop zeros](../research/flight-log-analysis/figures/turn-model-spike/classical-lateral/annotated-root-locus.png)

*Crosses are open-loop poles, circles are combined-loop zeros, and green squares are the selected closed-loop poles. The mode labels come from eigenvector participation: several motions mix pitch, yaw, and translation. The red star marks the gain-induced instability boundary. The lower-half-plane branches mirror the upper half.*

The slow oscillatory plant mode mixes pitch and forward speed at about 0.34 Hz. Another mixes sideways velocity and canopy yaw around 0.59 Hz; the fastest pair is predominantly relative canopy/payload roll at 5.86 Hz. Feedback changes these coupled motions rather than merely adding damping to a single isolated roll mode.

Zeros need a different interpretation: they describe cancellation in an input–output response, not another natural motion. The combined loop has a right-half-plane zero pair at $(6.944\pm j19.054)\,\mathrm{s}^{-1}$. The individual heading-rate and roll-rate plants have different zeros, so those cannot be substituted as endpoints of this root locus. The output-specific maps below show the distinction. These are properties of the modeled plant, not modes or zeros identified directly from flight.

![Heading-rate and roll-rate plant poles and zeros](../research/flight-log-analysis/figures/turn-model-spike/classical-lateral/plant-zero-map.png)

### Shape feedback without sacrificing feedforward

The filter comparison keeps every gain and model coefficient fixed and changes only measurement filtering. It compares six filters across level flight, climb and descent at ±0.5 m/s, and level turns at ±10 deg/s. Two candidates stood out: lowering the existing low-pass cutoff from 4 to 1 Hz, and adding a broad notch at 2.5 Hz.

$$
\begin{aligned}
H_1(s) &= \frac{6.283}{s+6.283}\quad\text{(1 Hz candidate)} \\
H_N(s) &= H(s)\frac{s^2+6.283s+246.740}{s^2+21.991s+246.740}\quad\text{(notch candidate)}
\end{aligned}
$$

Both retain unity DC gain. Feedforward bypasses the measurement filters, so reducing feedback near the troublesome modes need not make every turn command sluggish.

| Measurement filter | Level-flight gain margin | Lowest margin across five trims |
|---|---:|---:|
| Existing 4 Hz low-pass | 3.66 dB | 2.31 dB |
| 1 Hz low-pass | 8.41 dB | 6.87 dB |
| Broad 2.5 Hz notch | 14.83 dB | 7.04 dB |

![Feedback magnitude and sensitivity with six measurement filters](../research/flight-log-analysis/figures/turn-model-spike/lateral-shaping/shaping-bode.png)

*Both candidates reduce the sensitivity peak near the troublesome mode. The notch has the strongest straight-flight margin, but its advantage over the simpler filter narrows in left turns.*

Local command tracking stayed similar. Eighteen nonlinear tests check that result: level flight and both turn directions, each with three filters and either zero or 80 ms of added sensing delay. These used the actual modeled brake travel limits, asymmetric PWM mapping, changing speed scaling, and integral limits. The requested increments were 2 deg/s, continuing in the existing turn direction.

![Nonlinear command tracking and brake effort with zero and 80 ms sensing delay](../research/flight-log-analysis/figures/turn-model-spike/lateral-shaping/nonlinear-shaping.png)

*Solid lines have no added delay; dashed lines have 80 ms. The existing filter develops a large oscillation in the right turn, while both candidates keep tracking. The left-turn baseline remains well behaved, consistent with the asymmetric plant.*

In the delayed right-turn case, final-ten-second heading-rate RMS error was 26.65 deg/s with the existing filter, versus about 0.016 deg/s with either candidate. Without added delay, the 1 Hz filter’s integrated right-turn tracking error changed from 5.10 to 5.12 degrees over the 30-second run. The benefit is more tolerance of feedback phase lag, with little tracking cost in these tests.

### What the native 1 Hz experiment establishes

The native implementation trial includes the full ArduPlane controller, estimator, navigation and altitude loops together. It compares `PG_TURN_FILT = 4` and `1` Hz in calm air, moderate repeatable gusts, and a steady 2 m/s eastward wind. All gains and model coefficients stayed fixed. The 1.758 kg Trout Lake model flew the same mission at 22.2 V, including altitude changes and clockwise and counterclockwise loiters. **All six native AUTO tests passed.**

| Condition | Right-turn heading-rate RMS error, 4 → 1 Hz | Left-turn heading-rate RMS error, 4 → 1 Hz |
|---|---:|---:|
| Calm | 0.36 → 0.26 deg/s | 0.39 → 0.31 deg/s |
| Moderate gusts | 9.93 → 9.66 deg/s | 9.43 → 10.44 deg/s |
| Steady crosswind | 2.39 → 1.93 deg/s | 3.75 → 3.25 deg/s |

These errors compare the requested rate with the unfiltered simulated payload heading rate, rather than rewarding the filter for making its own feedback signal smoother. Across the six settled-loiter comparisons, brake-command content above 1 Hz fell by **34–69%**. Radial tracking stayed essentially unchanged or improved slightly. In gusts, altitude RMS error remained around 0.41 m, and none of the settled loiters saturated the differential-brake command.

![Native 4 Hz versus 1 Hz filter comparison: heading-rate error, radial error and rapid brake-command activity](assets/lateral-native-filter/metrics.png)

*Blue is 1 Hz. Quieter commands are a consistent benefit; better heading-rate tracking is not universal. In particular, the gusty left turn had higher rate error even though radial tracking improved.*

![Native AUTO loiter-entry responses in calm air, gusts and crosswind](assets/lateral-native-filter/comparison.png)

*Matched settings, with each first-loiter entry aligned independently. Most of the remaining gust-driven rate error is below 1 Hz. Reducing high-frequency feedback does not remove that slower motion.*

**The 1 Hz filter is the preferred lateral candidate for the next combined SITL comparison, with the gains unchanged.** The selected full-loop longitudinal study retains the 4 Hz steering baseline so that its results isolate the climb-PI change; the two improvements have not yet been validated together. This is an incremental improvement, not a cure for gust response. These are single paired runs; the repeatable gust field has a different arrival phase when the trajectory changes. They also do not reproduce the earlier artificial 80 ms delay test or remeasure a stability margin. The existing mission replays and published default parameters still use 4 Hz. [Data and methods](data-and-methods.md#native-lateral-filter-trial) records the exact configurations, analysis and scope.

## Altitude: keep TECS's framework, use throttle as the actuator

![Standard TECS architecture](assets/control-loops/tecs-standard.png)

Standard TECS coordinates total energy and its distribution between height and airspeed using throttle and pitch demand. This parafoil has no elevator-like pitch actuator. Continuous symmetric braking would cost efficiency and wing stability. The adaptation treats airspeed as approximately trimmed and controls vertical speed through throttle.

![Parafoil altitude controller](assets/control-loops/tecs-parafoil.png)

The outer loop adds an altitude-error correction to the altitude trajectory's rate of change:

$$
v_{z,c}=\dot h_c+\frac{h_c-h}{T_h}
$$

The climb demand is bounded. Throttle feedforward estimates the required power, PI feedback corrects climb-rate error, and filtered payload pitch rate supplies a separate damping correction:

$$
\begin{aligned}
u_T &= u_{\mathrm{FF}}(v_{z,c})+K_Pe_v \\
&\quad+K_I\int e_v\,dt-K_qF_q(s)q_{\mathrm{payload}}
\end{aligned}
$$

### Calibrate feedforward before increasing feedback

At 22.2 V, model trim requires about 44.6% throttle for level flight and 59.9% for a 1 m/s climb. Development settings use 45% trim, 60% maximum throttle, a 1 m/s climb calibration and a 2.5 m/s descent calibration. These calibrate the simulator's local throttle map; they do not replace flight-derived performance estimates.

~~~text
Climb:   u_FF = trim + (v_cmd / climb_limit) × (maximum − trim)
Descent: u_FF = trim + (v_cmd / sink_limit) × (trim − minimum)
~~~

ArduPlane's existing battery compensation scales output as voltage falls. The analysis includes that scaling and the compensated throttle limit returned to TECS. The integrator should not have to compensate for a battery path that was accidentally disabled.

A 0.5 s first-order filter smooths the trajectory derivative used for feedforward. Height feedback and climb/sink limits remain intact. This prevents abrupt altitude capture from becoming a throttle kick.

### Shape damping around the modes

The original ACRA-inspired damper assumed that adding throttle would oppose a nose-down rotation. Thrust above the payload CG initially pitches this payload down. A fast minus-sign damper can reinforce the motion it is intended to suppress.

The [free-body diagram and conceptual animation in part 1](first-flight-controller.md#the-damper-that-could-make-things-worse) show the original assumption and the independent payload response.

### Why reversing the damper was only half the answer

The later simulation reproduced the distinction the flight had exposed: the immediate payload rotation and the aircraft’s eventual climb are different responses. I first tested the obvious correction—reverse the damper sign. It suppressed the fast pitching but could excite slower motion. Disabling the damper, reversing it, and filtering the original sign therefore became separate design alternatives, not steps toward an assumed correct sign.

The sign-comparison and altitude-command animations below show that result directly. The useful design retained negative pitch-rate feedback but reduced its action around the troublesome fast modes with two 1.5 Hz first-order low-pass stages:

$$
\Delta u_T(s)=-0.1\left(\frac{2\pi(1.5)}{s+2\pi(1.5)}\right)^2q_{\mathrm{payload}}(s)
$$

The combined filter reaches −3 dB near 0.97 Hz. Washout is disabled. The correction is bounded to ±0.25 throttle fraction before final throttle limits. This is a frequency-dependent design choice, not a claim that the original fast feedback was correct.

<a id="the-classical-analysis-that-sets-the-tuning"></a>

## Longitudinal classical analysis: tune the coupled altitude loop

The full-loop analysis keeps the motor, steering feedback, altitude loop, climb PI and pitch-rate filters together. Seven cases span level flight, climb, descent, lower voltage, a softer roll model and both feedforward slopes. Each is checked at three frozen lateral airspeed scales. A further sensitivity check varies the rotor response rate by ±30%.

Transfer functions come from the linearized state-space model. Numerical results retain poles and modal residues rather than expanding an ill-conditioned high-order polynomial:

~~~text
G_v(s) = C_v (sI − A)⁻¹ B       throttle → upward velocity
G_q(s) = C_q (sI − A)⁻¹ B       throttle → payload pitch rate
G(s)   = Σ residue_j / (s − pole_j)
~~~

![Throttle-to-climb and throttle-to-pitch frequency response](assets/longitudinal-final-design/plant-frequency.png)

With steering closed at nominal level trim, important plant modes include:

| Mode | Poles, s⁻¹ | Design implication |
|---|---|---|
| Slow pitch / forward-speed motion | −0.307 ± j2.152 | About 0.34 Hz; sustained gust response |
| Relative-pitch / vertical motion | −0.969 ± j15.978 | About 2.54 Hz; constrains aggressive feedback |
| Relative pitch and body rates | −5.117 ± j25.402 | About 4.04 Hz; gyro motion is not a single mode |
| Rotor response | −56.94 | Motor dynamics remain part of the plant |

These labels describe dominant participating states, not independent subsystems. The model also contains roll/yaw modes and controller-filter poles.

![Annotated plant poles and output-dependent transmission zeros](assets/longitudinal-final-design/plant-poles-zeros.png)

Zeros depend on the controlled output. The payload pitch-rate transfer has a real right-half-plane zero at +1.543 s⁻¹, consistent with its inverse response. With steering closed, the climb transfer includes right-half-plane zero pairs near +0.193 ± j13.086 and +3.014 ± j29.257 s⁻¹. They constrain high-frequency inversion; they do not imply that low-frequency climb has the same initial wrong-way response as payload pitch. A zero describes cancellation among paths, not an extra oscillation mode.

### Optimize the whole loop

The bounded optimization explored three directions: pitch-damper gain and filter shape, climb-rate PI strength, and altitude-loop speed. Ten configurations were screened across the operating cases; useful alternatives then faced matched nonlinear gust histories.

![Full-loop gain and sensitivity comparisons](assets/longitudinal-final-design/loop-frequency.png)

There are several unity-gain crossings. One phase-margin number would hide that structure. The study retains all crossings, examines the Nyquist locus and checks full closed-loop eigenvalues. Sensitivity peaks show where disturbances or model errors can be amplified; they do not replace nonlinear response checks.

![Damper gain root locus](assets/longitudinal-final-design/root-locus.png)

Lowering both pitch-filter cutoffs to 1 Hz improved the worst linear sensitivity peak but produced more pitch motion in both nonlinear gust realizations. Raising them to 2 Hz destabilized two screened cases. Larger damper gains also had an unfavorable motion/effort tradeoff. The pitch settings remain gain 0.1 with two 1.5 Hz stages.

The useful change is stronger climb-rate PI:

~~~text
K_P = positive_throttle_range / (TECS_TIME_CONST × TECS_VERT_ACC)
K_I = TECS_INTEG_GAIN × K_P
~~~

Changing TECS_VERT_ACC from 7 to 3.5 doubles both gains while preserving their ratio. At nominal voltage, K_P becomes 0.03429 throttle fraction per m/s and K_I becomes 0.01029 per metre. This parameter is the gain-scaling denominator in the paraglider path; it is not a newly measured physical acceleration limit.

The outer height time constant remains 10 s, derivative-feedforward smoothing remains 0.5 s, and feedforward calibration is unchanged. Faster altitude feedback improved the small-command tracking score but did not address gust disturbances as directly.

<a id="what-the-selected-tuning-achieves"></a>

## What the selected longitudinal tuning achieves

For a 0.1 m altitude ramp over two seconds, linear prediction differs from the nonlinear candidate by about 0.2 mm height RMS and 0.011°/s pitch-rate RMS. The nonlinear diagnostic includes discrete updates and piecewise feedforward, using simulated truth rather than the native estimator.

![Linear predictions and nonlinear small-command response](assets/longitudinal-final-design/full-loop-validation.png)

Matched gust histories show the tradeoff:

| Metric | Original gusts: baseline → selected | Different, 50% stronger gusts: baseline → selected |
|---|---:|---:|
| Height error RMS | 0.404 → 0.365 m | 0.531 → 0.480 m |
| Climb-rate RMS | 0.455 → 0.420 m/s | 0.544 → 0.506 m/s |
| Payload pitch standard deviation | 6.17 → 6.24° | 10.14 → 10.16° |
| Pitch-rate RMS | 18.03 → 18.77°/s | 25.08 → 25.58°/s |
| Throttle standard deviation | 2.65 → 3.07 percentage points | 3.91 → 4.43 percentage points |

The choice prioritizes altitude and climb control while accepting modestly more rate motion and throttle effort. No matched run saturates throttle. This is a useful refinement, not a dramatic change in the forced airframe response.

### Native AUTO verification

The selected configuration passed the existing full AUTO test in calm air and synthetic gusts, including altitude changes, both opposing two-turn loiters, home loiter and controller handover. Recorded parameters confirm that only TECS_VERT_ACC changed from the preceding baseline; the backend reported no errors or clipped packets.

![Native AUTO comparison of baseline and selected climb-rate gains](assets/longitudinal-final-design/native-comparison.png)

| Native result | Baseline → selected |
|---|---:|
| Gusty CW loiter height RMS | 0.408 → 0.375 m |
| Gusty CCW loiter height RMS | 0.435 → 0.379 m |
| Calm CW loiter height RMS | 0.064 → 0.053 m |
| Calm CCW loiter height RMS | 0.080 → 0.089 m |
| Gusty maximum waypoint height error | 2.26 → 2.20 m |
| Calm maximum waypoint height error | 1.71 → 1.66 m |
| Gusty maximum corner overshoot | 1.39 → 1.35 m |
| Calm maximum corner overshoot | 0.36 → 0.63 m |

Settled gusty pitch-rate RMS rises from 19.02 to 20.11°/s clockwise and 18.53 to 20.32°/s counterclockwise. Throttle standard deviation rises from 2.90/2.81 to 3.42/3.42 percentage points. These costs accompany the altitude improvement.

The baseline is the archived passing two-stage-damper run; the candidate uses the same route, model and gust generator. Arrival times differ, so the loiters encounter different gust phases. The matched nonlinear histories isolate the tuning change; the native tests establish integration behavior, rather than statistical superiority across weather.

The [feature branch’s paired mission-development parameter files](https://github.com/DavidIngraham/ardupilot/commit/5f483ade60) now select TECS_VERT_ACC = 3.5. This changes development tuning, not ArduPilot's generic parameter default. The new low-voltage and softer-roll results are local linear checks; the fresh native runs here use the nominal model at 22.2 V.

## Seeing why the damping path matters

These interactive diagnostics retain the earlier climb-feedback setting to isolate the damper configurations. They explain the architecture; they are not recordings of the doubled-PI candidate.

[Compare damper off, original sign, reversed sign and filtered feedback](https://davidingraham.github.io/assets/microagu-viewer/pitch-comparison.html)

*Identical small pitch-rate disturbances: damper off, fast original sign, reversed sign, and two-stage filtering. The camera preserves carriage translation and rotation. Standalone truth-sensed simulation.*

[Compare the four dampers during an altitude command](https://davidingraham.github.io/assets/microagu-viewer/altitude-comparison.html)

*A 100 → 105 m altitude command separates the fast-sign problem from slower instability. Off and two-stage-filtered cases settle; simply reversing the fast damper does not. These are diagnostic alternatives, not flight recordings.*

## The controller flying the aircraft

The calm, gust and crosswind replays below show native AUTO with the same two-stage damper and the preceding climb-feedback gains. They retain the CAD payload, articulated suspension, rotating capstans and propeller, throttle history, brake pull, wind indicator, ground track and altitude side profile. The numerical comparisons above document the subsequent PI refinement.

**Calm air**

[Replay native AUTO in calm air](https://davidingraham.github.io/assets/microagu-viewer/simulation-replay.html?case=calm)

**Moderate synthetic gusts**

[Replay native AUTO in synthetic gusts](https://davidingraham.github.io/assets/microagu-viewer/simulation-replay.html?case=gust)

**Steady 2 m/s eastward wind**

[Replay native AUTO in steady crosswind](https://davidingraham.github.io/assets/microagu-viewer/simulation-replay.html?case=crosswind)



## Bringing the two axes together

The lateral analysis supports strong feedforward, modest heading-rate feedback and less measurement feedback near the coupled modes. The longitudinal analysis supports calibrated throttle feedforward, two-stage pitch filtering and stronger climb-rate PI. Both designs separate command tracking from damping instead of asking a large feedback gain to do everything.

The remaining integration experiment is specific: compare the 1 Hz lateral candidate against the 4 Hz baseline with the selected doubled climb PI, using the same native missions and wind cases. The separate successful trials motivate that combination; they do not establish its combined performance.

## Scope and next steps

This is an up-and-away simulator controller, evaluated on a model informed by the flights. Original flight-tested settings remain distinct from new SITL settings. Launch, canopy collapse, landing flare and flight validation are outside this study. Saturation, estimator delay and model error remain reasons to check nonlinear and native behavior after linear design.

A relative-motion observer remains a possible extension. The present result uses the existing payload gyro and navigation estimates. Next comes aircraft validation and, separately, testing the lateral 1 Hz candidate alongside this longitudinal tuning.

[Data and methods](data-and-methods.md#full-loop-longitudinal-design) records reproduction commands, numerical results, parameter provenance and the archived experimental sequence. Raw trajectories and flight logs remain in Mission Planner.

The [next flight test card](next-flight-test-card.md) turns that validation into a staged sequence, with configuration records, ordinary throttle/turn maneuvers, takeover criteria and a postflight analysis plan.
