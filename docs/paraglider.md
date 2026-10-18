# Steering a Parafoil with an Airplane Autopilot

This earlier axis-specific page is retained for existing links. Read the consolidated story in [part 1: through Trout Lake](first-flight-controller.md) and [part 2: the subsequent controller design](controller-design.md).

*Micro-AGU · Flight-tested lateral background · [Start with the flight story](flight-testing.md) · [Design](micro-agu-design.md) · [Steering](paraglider.md) · [Throttle and pitch](longitudinal-observer.md) · [Final controller](controller-design.md) · [Data and methods](data-and-methods.md)*

I initially thought I would need to design a yaw-rate controller for the parafoil. Differential braking makes it turn, so controlling the rate of that turn seemed more natural than asking an airplane roll controller to do the job. I wanted to keep ArduPlane's navigation and replace the part that translated its demands into brake movement.

As I worked through my notes, I figured out that I could get much of the same behavior by changing the gains in the controller that was already there. The trick was to turn off the inner roll-rate feedback and use its feed-forward path as an angle gain. With bank angle standing in for turn rate, that gave me a practical route to the first flights without writing the new controller immediately.

The flights got me to working autonomous missions. Later, a model with separate canopy and payload motion exposed how steering corrections could excite the suspended payload, and gave me a way to tune turn tracking and damping separately. The [consolidated controller article](controller-design.md) explains the dedicated turn-rate design developed afterward. The [flight-test post](flight-testing.md) covers the aircraft tests; this page follows that progression in the lateral controller.

## Getting the first controller by changing the gains

I started by breaking ArduPlane's roll controller into its individual paths. It normally converts bank-angle error into a requested roll rate, then uses rate feedback and feed-forward to drive the ailerons. I crossed out the rate P, I, and D paths and followed what was left. The feed-forward path reduced to something much simpler: essentially a proportional gain on bank-angle error.

![Original lateral-control analysis comparing ArduPlane, the first-flight configuration, and the simplified angle controller](assets/micro-agu/lateral-control-notes.png)

*January 30 control-law notes: keep the navigation and angle-error calculation, then use rate feed-forward to drive the brakes.*

![Original nested bank-angle and roll-rate loops](assets/control-loops/original-roll.png)

*The starting architecture: bank-angle error generates a roll-rate target; rate feedback and feedforward drive the actuators. This diagram omits implementation details to show the two loops.*

A conventional airplane roll controller has two nested loops. Bank-angle error creates a requested roll rate, then a rate controller drives the ailerons. For my parafoil, the output instead goes through the brake mixer. Differential braking produces a turn and the associated bank, so I wanted a simple angular correction rather than an inner loop tuned around aileron-driven roll dynamics.

Let $\phi_{\mathrm{cmd}}$ be commanded bank, $\phi$ be measured bank, $\tau$ be `RLL2SRV_TCONST`, and $K_{\mathrm{FF}}$ be `RLL_RATE_FF`. Ignoring the target filter and limits for the moment, ArduPlane's outer loop computes:

$$
\begin{aligned}
e_\phi &= \phi_{\mathrm{cmd}}-\phi \\
p_{\mathrm{cmd}} &= \frac{e_\phi}{\tau}
\end{aligned}
$$

The rate feed-forward term is proportional to that requested rate. With rate P, I, D, and derivative feed-forward set to zero, it is the only active rate-controller contribution:

$$
\begin{aligned}
u &= K_{\mathrm{FF}}p_{\mathrm{cmd}} = \frac{K_{\mathrm{FF}}}{\tau}(\phi_{\mathrm{cmd}}-\phi) = K_\phi e_\phi \\
K_\phi &= \frac{K_{\mathrm{FF}}}{\tau}
\end{aligned}
$$

**The rate feed-forward gain becomes a proportional bank-angle gain.** It is feed-forward with respect to the inner rate loop, but the complete loop still feeds back measured bank angle. There is no integration hidden in the cancellation: the requested rate is just angle error multiplied by $1/\tau$. Measured roll rate no longer contributes when its feedback gains are zero.

![Flight-tested angle-control workaround using rate feedforward](assets/control-loops/flight-angle-workaround.png)

*The flight configuration removes the measured-roll-rate inner loop. Rate feedforward acts on bank-angle error through the outer angle gain. Trout Lake adds derivative feedforward to this same path.*

### Including ArduPilot's speed scaling

The implementation scales the PID inputs and rescales feed-forward on the way out. Let $S$ be the supplied speed scaler, $E$ be the true/equivalent airspeed ratio (`EAS2TAS`), and $F$ be the feed-forward multiplier (`ff_scale`, normally 1). With angles expressed in degrees:

$$
\begin{aligned}
\text{PID target} &= \operatorname{radians}(p_{\mathrm{cmd}})S^2 \\
\text{PID FF} &= K_{\mathrm{FF}}\operatorname{radians}(p_{\mathrm{cmd}})S^2 \\
u_{\mathrm{FF}} &= \operatorname{degrees}\!\left(\frac{F\,\text{PID FF}}{SE}\right) \\
&= F K_{\mathrm{FF}}\frac{S}{E}p_{\mathrm{cmd}} = \frac{F K_{\mathrm{FF}}S}{E\tau}(\phi_{\mathrm{cmd}}-\phi) \\
K_\phi &= \frac{F K_{\mathrm{FF}}S}{E\tau}
\end{aligned}
$$

The radians/degrees conversions cancel, as does one factor of $S$. The nominal speed scaler is reference speed divided by estimated airspeed; the actual supplied scaler includes ArduPlane's bounds and fallback behavior. At fixed scaling, feed-forward is still simply an angle gain. The final controller output is converted to centidegrees, limited to ±4500, and passed to the mixer—these are controller units, not measured degrees of brake travel.

The Hood River log records the configuration that implements this idea:

| Parameter | Logged value | Role |
|---|---:|---|
| `RLL2SRV_TCONST` | 1.0 s | Converts bank error to rate demand |
| `RLL_RATE_FF` | 0.345 | Supplies the effective angle gain |
| `RLL_RATE_P`, `RLL_RATE_I`, `RLL_RATE_D` | 0 | Removes inner rate-error feedback |
| `RLL_RATE_D_FF` | 0 | No derivative feed-forward in this flight |
| `RLL2SRV_RMAX` | 0 | Disables the requested-rate limit |
| `RLL_RATE_FLTT` | 3 Hz | Filters the rate target before feed-forward |

For example, with $S=E=F=1$, a 10° bank error produces a 10°/s rate request and a feed-forward output of 3.45 controller degrees, or 345 centidegrees. Changing FF changes the angular correction directly; halving $\tau$ doubles it.

The 3 Hz target filter means the dynamic implementation is a filtered angle controller. At constant speed scaling, with $H_T(s)$ representing that filter:

$$
\begin{aligned}
u_{\mathrm{FF}}(s) &= K_\phi H_T(s)\bigl[\phi_{\mathrm{cmd}}(s)-\phi(s)\bigr]
\end{aligned}
$$

### Where damping fits

I also worked through how I could add damping using the derivative feed-forward path. If derivative feed-forward is enabled, it differentiates the requested rate—which already contains bank-angle error. At fixed scaling, its contribution reduces to:

$$
\begin{aligned}
u_{\mathrm{DFF}} &= \frac{F K_{\mathrm{DFF}}S}{E\tau}\frac{d(\phi_{\mathrm{cmd}}-\phi)}{dt}
\end{aligned}
$$

For a steady bank command this becomes a negative bank-rate term, supplying damping. During a changing command it also reacts to the commanded bank rate. This is different from the inner rate-P term, which acts on requested minus measured body roll rate. I left D_FF at zero for the January flight, but this gave me a way to add damping to the same basic structure.

ArduPlane's navigation still supplies the bank command from desired lateral acceleration:

$$
\begin{aligned}
\phi_{\mathrm{cmd}} &= \arctan\!\left(\frac{a_{\mathrm{lat,cmd}}}{g}\right)
\end{aligned}
$$

### Why this was close to the yaw-rate controller I wanted

The other part of the idea was the relationship between bank and turn rate. For a steady coordinated turn at airspeed $V$, heading rate $\omega$ is:

$$
\begin{aligned}
\omega &= \frac{g}{V}\tan\phi
\end{aligned}
$$

At small bank angles, $\tan\phi\approx\phi$, with $\phi$ in radians. At fixed speed, the commanded and actual bank angles therefore give:

$$
\begin{aligned}
\omega_{\mathrm{cmd}} &\approx \frac{g}{V}\phi_{\mathrm{cmd}} \\
\omega &\approx \frac{g}{V}\phi \\
\phi_{\mathrm{cmd}}-\phi &\approx \frac{V}{g}(\omega_{\mathrm{cmd}}-\omega)
\end{aligned}
$$

Substituting that into the angle controller gives:

$$
\begin{aligned}
u &\approx K_\phi\frac{V}{g}(\omega_{\mathrm{cmd}}-\omega) = K_\omega(\omega_{\mathrm{cmd}}-\omega) \\
K_\omega &= \frac{K_\phi V}{g}
\end{aligned}
$$

That was the useful discovery: **under the steady-turn approximation, the angle loop is mathematically equivalent to a proportional turn-rate loop with a rescaled gain.** I could use the bank-angle demand already produced by navigation, adjust the existing gains, and get something close to the yaw-rate behavior I had set out to implement.

Here $\omega$ means heading rate, the rate of turning in the horizontal plane. It is the navigation quantity I wanted to control; it is not generally identical to the body-axis yaw gyro reading. The equivalence uses steady coordinated motion and fixed speed. Around a larger steady bank angle $\phi_0$, the local conversion becomes $K_\omega=K_\phi V\cos^2(\phi_0)/g$.

## Hood River → simulation → Trout Lake

The first sustained flight at the Barrett Park RC field in Hood River used FF = 0.345 and no derivative feed-forward. Before Trout Lake, I tuned the workaround in our simple initial simulation model. Mission Planner recorded the experiments on February 14 and the morning of February 15, including trials with rate feedback, different angle time constants, and a wide range of feed-forward gains.

![Gain experiments and simulated response on the morning before Trout Lake](assets/flight-testing/sitl-lateral-tuning.png)

*These are recorded simulation runs. The mode trace identifies changes between GUIDED, AUTO and MANUAL while I adjusted the controller.*

The close-up below shows two AUTO runs from that morning. At FF = 2.0 and D_FF = 0.10, the simulated bank swings much more widely than in the selected FF = 1.0, D_FF = 0.05 run. Both traces also show that measured bank does not simply follow the navigation bank demand. The initial model was useful for exploring this behavior and choosing gains; flight testing was the next step.

![Commanded and simulated bank, with cross-track error, in two AUTO tuning windows](assets/flight-testing/sitl-bank-tracking.png)

*Two 45-second excerpts on February 15: 11:30:00 and 11:35:15 PST. The axes use the same scale. These are different portions of AUTO navigation, with different command histories, rather than identical repeated step inputs.*

By 11:37 a.m. I had returned to FF = 1.0 and D_FF = 0.05. I also used a one-second angle time constant, a 1 Hz target filter and a 30 deg/s requested roll-rate limit. Rate P, I and D were zero again. Compared with Hood River, the nominal angle gain was 2.9 times larger, with derivative damping added.

At my friend Collin’s house in Trout Lake that afternoon, I flew those settings without changing the lateral gains during the long flight. I deliberately reduced the loiter-radius setting from 60 to 40 metres, then to 30 metres, to explore tighter turns. The [flight-test article](flight-testing.md) shows the measured response to the first reduction. That sequence—first flight measurements, simulation tuning, then another flight—gave me a practical controller and evidence for what to improve next.


## From flight-tested workaround to dedicated control

The real flights used the roll-loop adaptation above. The [final controller design](controller-design.md) develops the dedicated heading-rate loop and throttle controller together, explains their classical tuning, and presents the native mission results.

<a id="why-i-came-back-to-a-dedicated-turn-rate-controller"></a>
<a id="tuning-the-dedicated-loop-with-separate-canopy-and-payload-motion"></a>
<a id="the-current-loop-one-operation-at-a-time"></a>
<a id="a-mission-that-tracks-without-the-repeated-rocking"></a>
<a id="what-the-linear-analysis-tells-me"></a>
<a id="writing-the-feedback-loop-explicitly"></a>
<a id="the-poles-explain-why-more-p-was-not-helping"></a>
<a id="less-feedback-at-the-troublesome-frequencies"></a>
<a id="taking-the-1-hz-candidate-through-native-sitl"></a>
<a id="watching-the-two-bodies-move"></a>
<a id="the-controller-flies-the-mission"></a>

The [post-flight research record](controller-research-record.md#lateral-experiments) retains detailed experiments and original figures. The [controller article's flight videos](controller-design.md#the-controller-flying-the-aircraft) show calm, gusty and crosswind AUTO; the scripted articulation clip remains below.

[Replay the powered turn and reversal](https://davidingraham.github.io/assets/microagu-viewer/simulation-replay.html?case=turn)

*Scripted inputs in the 9-DOF simulator, with CAD payload articulation. This demonstrates the viewer; it is not an AUTO mission. [Viewer documentation](../viewer/README.md).*
