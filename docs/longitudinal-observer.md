# Throttle, Pitch, and the Controller That Pushed the Wrong Way

This earlier axis-specific page is retained for existing links. Read the consolidated story in [part 1: through Trout Lake](first-flight-controller.md) and [part 2: the subsequent controller design](controller-design.md).

*Micro-AGU · Flight-tested longitudinal background · [Start with the flight story](flight-testing.md) · [Design](micro-agu-design.md) · [Steering](paraglider.md) · [Throttle and pitch](longitudinal-observer.md) · [Final controller](controller-design.md) · [Data and methods](data-and-methods.md)*

ArduPlane already had a controller for managing altitude and airspeed: TECS, the Total Energy Control System. I wanted to keep its connection to the navigation and flight-mode machinery, but change how it controlled the aircraft. A parafoil does not have an elevator, and the suspended payload's pitch is not something I can command like an airplane's attitude.

My first adaptation used throttle to control climb and descent while letting the canopy fly near its trimmed airspeed. I also borrowed a pitch-rate damper from the ACRA paramotor paper to suppress pitching during altitude changes. The altitude controller got me to autonomous missions; the damper exposed a mistake in how I pictured the aircraft's response to thrust. This page covers the original TECS changes and flight measurements. The [consolidated controller article](controller-design.md) develops the final frequency-shaped design and classical analysis.

## What TECS normally controls

[TECS](https://ardupilot.org/plane/docs/tecs-total-energy-control-system-for-speed-height-tuning-guide.html) coordinates throttle and pitch demands to manage height and airspeed. The useful starting point is the aircraft's specific energy: potential energy per unit mass is gh, and kinetic energy per unit mass is V²/2.

![Specific energy: E_T = gh + V²/2.](assets/control-loops/tecs-energy.png)

Throttle controls the rate at which energy is added to the aircraft. Pitch helps determine how that energy is divided between speed and height. Climbing by pitching up alone can spend airspeed; adding power allows a sustained climb without that continuing loss of speed. TECS coordinates those demands, and the airplane's pitch controller uses the elevator to follow the requested attitude.

![Standard TECS control loop: energy control drives throttle; energy balance commands pitch through the elevator.](assets/control-loops/tecs-standard.png)

That second actuator was the mismatch for my aircraft. Symmetric brake pull was not a reliable substitute for elevator-based pitch control. Using it continuously that way would compromise efficiency and wing stability. I wanted to reserve symmetric braking primarily for a landing flare, which I still intend to add. Landing was outside the scope of the initial project phases.

## Adapting TECS to throttle-only altitude control

In my [original paraglider TECS implementation](https://github.com/DavidIngraham/ardupilot/commit/8ed5b0689e31aa9a27fa2cf78057dce15f9b0d16), I replaced the kinetic-energy and energy-balance control with a vertical-speed controller. Pitch demand stayed at trim. Approximately constant airspeed was the operating assumption for this first version, not a claim that brakes, turns or gusts could never change it.

I retained TECS's altitude and climb-rate estimates, throttle limits and connection to the flight modes. The control path became:

1. Shape the requested altitude into a climb/sink-limited trajectory.
2. Convert altitude error into a vertical-speed demand, adding the trajectory's own rate of change.
3. Estimate the required throttle from trim and the configured climb/sink performance.
4. Correct vertical-speed error with proportional and integral feedback.
5. Add a separate pitch-rate damping correction.

![Parafoil TECS control loop with altitude and climb-rate feedback, throttle feedforward and a separate negative pitch-rate feedback path.](assets/control-loops/tecs-parafoil.png)

Let h<sub>c</sub> be the shaped altitude demand, h measured altitude, and v<sub>z</sub> measured upward velocity. Before the climb/sink limits, the outer loop requests

![Requested climb rate = altitude trajectory rate + altitude error / height time constant.](assets/control-loops/tecs-height.png)

The first term anticipates a moving altitude target; the second closes the remaining height error over the height-demand time constant T<sub>h</sub>. With e<sub>v</sub> = v<sub>z,c</sub> − v<sub>z</sub>, the inner control law can be summarized as

![Throttle = feedforward + proportional correction + integral correction − filtered pitch-rate damping.](assets/control-loops/tecs-throttle.png)

Here u<sub>T</sub> is throttle fraction, u<sub>FF</sub> is the climb/descent feedforward map, and F<sub>q</sub> filters the payload gyro's pitch rate. This equation shows the signal paths; the implementation also applies throttle limits, slew limiting to the proportional-plus-feedforward path, integrator bounds and a bounded damper correction.

The feedforward map was an important part of the design. I did not want the integrator to discover the throttle required for every sustained climb. Trim throttle supplied the level-flight baseline, while the configured climb and sink rates set the slopes toward the throttle limits. The flight measurements below let me calibrate that relationship.

The pitch-rate path had a different job: suppress oscillation while preserving a responsive altitude change. The ACRA paper describes stabilizing pitch during altitude control and reports reduced altitude oscillations without degrading transient response. That was the behavior I wanted. It was also where the difference between a rigid aircraft model and my suspended payload became important.

## The damper that could make things worse

I had added a pitch-rate damper to throttle. My initial intuition was that more throttle would pitch the model up, so adding throttle during a nose-down rotation should oppose that motion.

That idea came from Umenberger and Göktogan’s [ACRA 2012 paramotor paper](https://www.araa.asn.au/acra/acra2012/papers/pap151.pdf). Figure 6 feeds pitch rate back into throttle, and Figures 8–9 show a simulated nose-up throttle-step response and improved damping. But §2.1 treats canopy and fuselage as one rigid body, while §2.2 holds their relative pitch fixed. That removes the independent payload-pitch mode that matters here.

The distinction is in the model and the response being controlled. Their §5 pitch transfer function was fitted to their six-DOF simulation; the paper’s flight-data identification concerns lateral dynamics. I did not find evidence establishing that a physically locked suspension made that damper sign valid on their real aircraft. The feedback makes sense for their modeled nose-up response; importing that sign into my payload-gyro loop was the mistake.

The throttle damper used:

```cpp
throttle_correction = -pitch_damping_gain * filtered_payload_pitch_rate;
```

With positive pitch rate defined as nose-up, a nose-down rotation produces a positive throttle correction. That only gives the intended damping if the relevant throttle-to-pitch response has the sign I expected.

The articulated model showed why that assumption could fail. The thrust line is below the suspension point, but above the payload CG. Forward thrust therefore produces a nose-down moment about the payload CG. The suspension point moves with the system; treating it as a fixed pivot leaves out an important part of the dynamics.

![Free-body diagram of the payload and canopy, showing thrust above the payload CG and the reinforcing feedback sequence](assets/flight-testing/pitch-damper-fbd-pusher.png)

*The pusher thrust acts above the payload CG, creating a nose-down moment. The canopy and payload react through the suspension. Schematic, not to scale.*

[Explore the pitch mechanics](https://davidingraham.github.io/assets/microagu-viewer/pitch-fbd.html)

*Left: my original whole-aircraft picture, with relative pitch locked and thrust below an assumed combined CG. Right: the payload can pitch independently; thrust acts above its CG, and its initial response is nose-down while the upper attachment moves forward. The side view uses the carriage CAD with schematic suspension and canopy. Geometry, angles and timing illustrate the mechanism rather than reproducing a simulation trace. The arrows isolate the incremental thrust moment; suspension reactions and aerodynamic forces also enter the full dynamics. The locked connection illustrates my assumption, not the ACRA aircraft’s construction.*

The later analysis explained the reinforcing loop:

**Payload pitches down → controller adds throttle → payload pitches down harder.**

I did not work this out at the field. Fortunately, I decided to disable the damper while testing, and the aircraft immediately became quieter. The logs and simulation later explained why: I had designed the feedback around the wrong initial pitch response. A throttle increase can rotate the payload nose-down even while the aircraft’s longer-term response is to climb. The moving suspension and the canopy’s aerodynamic forces determine how that motion develops.

The flight data contains a particularly useful comparison. In CRUISE, I had already set the throttle P and I gains to zero. At 655.79 seconds after boot, I set the pitch damper to zero too. In the tightly bounded windows below, that was the only control setting changed.

![Pitch rate and throttle immediately before and after the pitch damper was disabled](assets/flight-testing/damper-off.png)

| February 15 comparison | Boot-time window | Payload pitch-rate RMS |
|---|---:|---:|
| Damper gain 0.10; throttle P = I = 0 | 635–654 s | 76.4 degrees/s |
| Damper gain 0; throttle P = I = 0 | 659–681 s | 22.7 degrees/s |

The fast oscillation reduced sharply. Later in the same flight, reintroducing the damper at 0.05 coincided with a stronger component around 1.5 Hz; reducing it to 0.02 brought the rate RMS back down. Those observations support the self-excitation concern.

The logs gave me two tasks: understand that coupled motion, and establish a sound throttle baseline before asking feedback to improve it.

## Measuring what throttle could do

I used live telemetry and later Hood River log analysis to choose climb and sink settings. At the beginning of Trout Lake, I repeated the throttle tests with the heavier configuration and used level-flight testing to settle on 43% trim throttle.

![Throttle and climb measurements from Hood River and early Trout Lake](assets/flight-testing/throttle-climb-fit.png)

*Local fits to steady 10-second MANUAL/FBWA windows put level throttle near 35.5% at Hood River and 42.7% at Trout Lake. The Hood River result includes the brake trim and false power limiting present that day.*

| Setting | Sustained Hood River flight | Trout Lake start | Trout Lake final |
|---|---:|---:|---:|
| Maximum throttle | 40% | 40% | 60% |
| Trim throttle | 30% | 30% | 43% |
| Climb limit | 2 m/s | 1.8 m/s | 1.8 m/s |
| Maximum sink | 2 m/s | 3 m/s | 3 m/s |

Early at Trout Lake I raised maximum throttle to 65% and trim to 50%, then brought them back to 60% and 43%. The climb and sink limits were already set before takeoff. I derived those limits from Hood River logs; the local steady-response fits above reconstruct the trim relationship, rather than those original limit-selection intervals.

In the paraglider TECS implementation, trim is the zero-climb baseline. Requested climb or descent interpolates throttle toward its upper or lower bound:

$$
\begin{aligned}
u_{\mathrm{ff}} &= \begin{cases}u_{\mathrm{trim}}+\dfrac{\dot{h}_{\mathrm{cmd}}}{\dot{h}_{\mathrm{climb,max}}}(u_{\max}-u_{\mathrm{trim}}), & \text{climbing}, \\
u_{\mathrm{trim}}+\dfrac{\dot{h}_{\mathrm{cmd}}}{\dot{h}_{\mathrm{sink,max}}}(u_{\mathrm{trim}}-u_{\min}), & \text{descending}.\end{cases}
\end{aligned}
$$

Here $u$ denotes throttle and $\dot h_{\mathrm{cmd}}$ is the signed climb demand; the climb and sink limits are positive magnitudes.

The limits therefore set feed-forward slopes as well as bounding the requested vertical speed. P and I correct the remaining climb-rate error. That baseline tuning was separate from the pitch damper, which responded to angular motion.


## From the flight damper to the final controller

The [controller design article](controller-design.md) brings the later lateral and longitudinal work together: the final loop architecture, full-loop frequency analysis, selected gains and native AUTO evidence. It includes the sign-comparison and altitude-command animations.

<a id="returning-to-native-mission-control"></a>
<a id="match-throttle-feedforward-to-the-simulated-aircraft"></a>
<a id="smooth-the-altitude-capture-kick"></a>
<a id="keep-the-existing-voltage-compensation-connected"></a>
<a id="why-reversing-the-sign-was-not-enough"></a>
<a id="commanding-a-climb-with-the-four-damper-configurations"></a>
<a id="what-the-gust-video-exposed"></a>
<a id="shaping-the-damper-around-the-gust-response"></a>
<a id="back-in-the-native-auto-mission"></a>
<a id="where-that-leaves-the-controller"></a>
<a id="watching-the-sign-problem-in-the-current-model"></a>

[Continue to the final controller and classical analysis](controller-design.md#the-classical-analysis-that-sets-the-tuning), or [read the full experimental record](controller-research-record.md#longitudinal-experiments).
