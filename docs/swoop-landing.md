# Toward a Precision Swoop Landing

The [spot-landing controller](spot-landing.md) gave me a useful starting point: fly an approach with the existing autopilot, then use symmetric brake for flare while retaining differential steering. I wanted to see how far that architecture could go toward a more interesting arrival: a tight turning descent, followed by a rollout and a precise landing with very little forward speed. Ideally, I could choose between a soft approach and something more dramatic.

The current simulator can demonstrate the turning descent and place both versions close to the target. It does not yet finish with the slow, settled touchdown I want. That distinction matters: arriving over the bullseye is only one part of landing well.

## Two approaches, the same flight controller

Both examples run native ArduPilot AUTO missions in the existing 9-DOF Trout Lake model. Waypoints define a right-angle approach, L1 supplies lateral guidance, the heading-rate controller steers with differential brake, and the parafoil version of TECS controls the descent with throttle. Native landing logic triggers the common-brake flare. The [landing architecture](spot-landing.md) remains intact; this study adds no firmware changes or MPC.

The soft version gives the turn more room. The dramatic version starts lower, allows a faster heading-rate demand, and temporarily shortens the L1 period to tighten the turn. Once estimated heading passes 65 degrees on the north-to-east turn, the test harness restores the ordinary 10-second period for final approach. That schedule uses the autopilot's estimated heading, not simulator truth. It is an experimental mission harness, not a new onboard landing mode.

### Soft approach

[Explore the soft turning approach](https://davidingraham.github.io/assets/microagu-viewer/landing.html?case=swoop-soft)

### Dramatic approach

[Explore the dramatic turning approach](https://davidingraham.github.io/assets/microagu-viewer/landing.html?case=swoop-dramatic)

These two recordings are in calm air. Drag to orbit and zoom around the aircraft; the moving floor, bullseye, ground track and altitude profile show the trajectory, while the brake, throttle, speed and sink displays show the control response. The motion comes directly from the simulator. Each replay ends when the payload center of gravity crosses the ground plane; ground contact, sliding and canopy collapse are not simulated.

| Setting | Soft | Dramatic |
|---|---:|---:|
| Entry / corner altitude | 25 / 10 m | 20 / 8 m |
| Distance to corner / final leg | 55 / 40 m | 45 / 40 m |
| Heading-rate demand limit | 25 deg/s | 45 deg/s |
| L1 period | 10 s | 6 s during the turn, then 10 s |
| Flare altitude | 1.1 m | 1.1 m |
| Common-brake target / steering reserve | 85% / 10% | 95% / 5% |
| Brake ramp | 100 percentage points/s | 100 percentage points/s |

The inner-loop gains, L1 damping of 1.0 and 1 Hz measurement filter stay the same. These are two tested presets, rather than a continuous style knob. The lower flare trigger and stronger brake pull are specific to this experiment; they do not replace the ordinary landing configuration.

## Precision is working; settling is not there yet

In calm air, the soft approach reaches about 11 degrees of bank and the dramatic approach about 19 degrees. The dramatic turn gains airspeed from roughly 3.7 to 4.5 m/s between headings of 30 and 90 degrees. That is a visible turning descent, but still close to ordinary flight speed rather than a sustained, high-energy dive.

| Calm-air result | Soft | Dramatic |
|---|---:|---:|
| Distance from target | 2.41 m | 2.28 m |
| Horizontal speed at ground crossing | 2.86 m/s | 2.41 m/s |
| Downward speed at ground crossing | 0.92 m/s | 1.13 m/s |
| Bank at ground crossing | 1.5 degrees | 0.5 degrees |
| Payload pitch rate at ground crossing | -17.2 deg/s | -32.3 deg/s |

![Turning approach tracks, altitude, flare velocities and remaining payload pitch motion](../research/archive/swoop-2026-10-13/styles.png)

The aircraft rolls nearly level before arrival, but level bank does not mean the payload has settled. The stronger flare reduces forward speed more, while leaving more pitch motion and a higher downward speed at the ground plane. My starting aspirations were a miss below 5 m, horizontal speed below 1 m/s and downward speed below 0.5 m/s. The position objective is met; the two speed objectives are not.

## Does it survive a change in conditions?

I froze both presets and ran four checks for each: calm air, gentle synthetic gusts, a steady crosswind, and a perturbed entry with slower actuators and command delay. The gust amplitude is 0.25 m/s horizontally and 0.0625 m/s vertically. The crosswind is a 1 m/s northward air velocity, perpendicular to the eastbound final approach. The entry case combines a 2 m lateral offset, 1 m extra height, 5 degrees of heading error, 5% extra speed, a 150 mm/s actuator rate and 60 ms of command delay.

![Final eight-case comparison of target error and ground-crossing speeds](../research/archive/swoop-2026-10-13/validation.png)

All eight runs arrive within 5 m of the target; the largest miss is 3.26 m. Horizontal speed ranges from 2.12 to 2.86 m/s and downward speed from 0.92 to 1.28 m/s. Payload pitch-rate magnitude reaches 33.7 deg/s. These checks show repeatable placement across a small set of conditions, not a statistical reliability estimate.

The dramatic gust and crosswind cases also spend about a quarter of the flare at the remaining differential-command limit. Pulling almost all the common brake leaves little steering reserve. The simulator retains those actuator limits rather than allowing full steering on top of full flare.

## A sustained spiral before the landing

I also wanted to descend through several tight turns, build speed, then straighten out for the flare. A sustained spiral gives the aircraft time to settle into the turn and tests whether the model can actually store useful speed for the landing.

[Explore the descending spiral and spot landing](https://davidingraham.github.io/assets/microagu-viewer/landing.html?case=spiral)

This calm-air replay shows the entire native AUTO maneuver: roughly 2.3 turns, a tangent exit, final approach and brake flare. The aircraft starts at 50 m. ArduPilot's existing **loiter-to-altitude** command requests a 6 m clockwise orbit down to 12 m, then waits for its heading-exit condition before advancing to LAND. The actual exit altitude and number of turns depend on that native logic.

During the orbit, L1 uses an 8-second period and the heading-rate demand limit is 65 deg/s. When the mission advances to LAND, the research harness restores the 10-second period and 25 deg/s limit. The target lies 40 m east and 6 m north of the orbit center. The inner-loop gains, aircraft mass, rigging and aerodynamic coefficients remain unchanged. Flare uses the same 95% common brake, 5% steering reserve, 100 percentage points/s ramp and 1.1 m trigger as the dramatic approach above.

| Final spiral result | Calm | Gentle gusts |
|---|---:|---:|
| Heading swept before LAND | 826 degrees | 832 degrees |
| Peak bank | 17.5 degrees | 18.4 degrees |
| Airspeed entering LAND | 4.24 m/s | 4.43 m/s |
| Airspeed at flare | 4.60 m/s | 4.57 m/s |
| Target miss | 2.15 m | 1.96 m |
| Horizontal speed at ground crossing | 3.21 m/s | 3.17 m/s |
| Downward speed at ground crossing | 0.81 m/s | 0.85 m/s |

The gust case uses the same frozen configuration with smooth gusts of 0.25 m/s horizontal amplitude and 0.0625 m/s vertically. Both cases reach the target area, but they still arrive with substantial forward speed and about 18 degrees of payload pitch. Two runs establish a useful demonstration, not a reliability estimate.

![Calm spiral: track, altitude, speeds, attitude and actuator response](../research/archive/spiral-swoop-2026-10-13/spiral06.png)

The key question is whether the spiral itself builds speed. I checked that separately with fourteen open-loop cases: six asymmetric brake pulls and a zero-extra-pull baseline in each of the two retained model profiles. Throttle stays idle, the brake pull is held long enough to settle, then released.

![Sustained spiral and brake-release response in both model profiles](../research/archive/spiral-swoop-2026-10-13/authority.png)

At the strongest tested pull, the mission model settles at **4.58 m/s airspeed**, compared with **4.55 m/s** in the matched idle-glide baseline. The alternative model gives 4.54 versus 4.56 m/s. Bank and sink increase, but horizontal speed decreases. Releasing the brake briefly raises airspeed to about 5 m/s. The baseline retains the level-trim brake positions and a small turning bias; it is not a separately solved straight-glide trim.

That explains the distinction visible in the replay: the model can make a descending spiral and a transient recovery arc, but this study does not demonstrate sustained high-energy acceleration. The native traces also contain brief throttle and pitch transients around altitude capture and rollout, so their 5.3–5.4 m/s speed peaks cannot all be attributed to energy gained in the spiral. I have kept the model unchanged rather than tuning its aerodynamics to produce the maneuver I hoped to see.

The [spiral study record](../research/archive/spiral-swoop-2026-10-13/README.md) includes the gentle-gust plot, all seven native attempts, configurations and scripts. Two early routes failed to reach LAND; those failures remain in the record alongside the five completed landings.

## What needs a better terminal controller?

The flare is currently a scheduled brake ramp. It can trade speed for lift and drag, but it cannot independently choose arrival position, forward speed, sink rate and payload swing phase. In the simulated response, the reduction in sink comes before the forward-speed minimum; by the time the aircraft has slowed further, downward motion and pitch motion have returned.

I screened 32 open-loop turn and flare combinations as well as the native mission runs. Those screens show useful transient braking, but no demonstrated near-stop touchdown. They do not prove that the existing architecture has reached its limit. The aerodynamic model is unchanged throughout this study, and its high-brake response is an extrapolation beyond the flight-test evidence. It includes a smooth lift rolloff with angle of attack, but no validated stall or canopy-collapse dynamics.

The next useful step is to isolate an unpowered recovery arc from TECS altitude recapture, then optimize the short terminal brake schedule while explicitly penalizing pitch motion and residual speed. A matched approach without the spiral will help distinguish energy gained in the turn from transition transients. A small terminal MPC controller is worth comparing against that schedule using the same entry states and actuator limits. The approach guidance and TECS can remain responsible for getting to the maneuver; the open question is how much prediction is needed for the last few seconds.

For now, this is a reproducible precision turning-landing demonstration and a clear target for further control work. A settled swoop still needs improvement in simulation, followed by measurements that establish whether the predicted high-brake response is real.

[Study scripts, configurations and reproduction instructions](../research/archive/swoop-2026-10-13/README.md) · [Complete final metrics](../research/archive/swoop-2026-10-13/results.json)

Implementation and analysis assisted by Codex.
