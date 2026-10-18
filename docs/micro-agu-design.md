# Building Micro-AGU

*Micro-AGU · [Start with the flight story](flight-testing.md) · [Design](micro-agu-design.md) · [Steering](paraglider.md) · [Throttle and pitch](longitudinal-observer.md) · [First controller](first-flight-controller.md) · [After Trout Lake](controller-design.md) · [Data and methods](data-and-methods.md)*

My interest in guided parafoils started when I was 18, helping run tests at Stara Technologies. My job mostly involved riding an ATV around the Yuma desert in 115°F heat, recovering hardware after test flights. Some landings required a wider search pattern than others, especially on flights where we learned a bit more than we had planned.

Stara was developing miniature autonomous parafoil systems. A [2002 company announcement](https://guidedparafoils.org/wp-content/uploads/2019/10/ProQuestDocuments-2019-10-06.pdf) describes a guidance unit weighing just over half a pound, combining GPS, a compass, and motor-driven steering under a small parafoil. Its later [Mosquito system](https://www.spacewar.com/reports/STARA_Technologies_Demos_UAV_Precision_Airdrop_Capabilities_For_US_Military.html) pursued precision delivery of small payloads from unmanned aircraft. I was helping execute tests, not designing those guidance systems, but the idea stayed with me.

There is already a rich little corner of the RC hobby devoted to powered paragliders, often called RC paramotors. Companies such as [Opale Paramodels](https://www.opale-paramodels.com/gb/) and [Hacker's Para-RC](https://www.hacker-motor-shop.com/Para-RC/Para-RC-gliders.htm?a=catalog&p=7836&shop=hacker_e) offer purpose-built canopies, motorized frames, miniature pilots and accessories. Around that hardware is a community with years of experience in rigging, launching and flying these aircraft. That meant I could build on an existing flying platform and learn from people who already understood its peculiarities.

The barrier for me had always been cost. I'd wanted to try a project like this for years, but the specialist canopies and powered setups represented more money than I wanted to put into an experiment. HobbyKing offered a much cheaper entry point. I hadn't been impressed by its original canopy, but the [V2's redesigned wing and improved suspension lines](https://www.hobbyking.com/en_us/h-king-paramotor-v2-w-led-light-bar-2400mm-pnf.html) made it worth another look.

That changed the economics of the project: I could start with an affordable commercial canopy and concentrate my effort on the payload, guidance and control. Modern open-source avionics supplied the other key ingredient. Together, they made it practical to revisit the small guided parafoils that had first caught my imagination at Stara.

That became **micro-AGU—Micro Aerial Guidance Unit**: an opportunity to explore how much better a small guided parafoil could perform with modern open-source avionics.

![The assembled micro-AGU payload, with its printed frame, brake actuators and propeller guard](assets/micro-agu/build-assembled.jpg)

*The assembled payload, with its battery compartment open.*

The powered paraglider gives me a repeatable way to fly the guidance hardware, collect data, and study the interaction between the canopy, payload, and controller. The longer-term goal is a small autonomous guided parafoil.

This post brings together the physical design, my original notebook, and what the early flights taught me. The [flight-test story](flight-testing.md) covers the launches, ballast changes, autonomous missions, and pitch-damper trouble in more detail.

## A commercial canopy and a custom payload

The canopy is a **HobbyKing V2**, a 2.4 m single-skin wing. [HobbyKing redesigned the wing and improved the suspension lines](https://hobbyking.com/en_us/h-king-paramotor-v2-w-led-light-bar-2400mm-pnf.html), with testing and feedback from JohnVHRC. Those changes made V2 worth trying.

![The custom payload resting on the folded canopy with suspension lines visible](assets/micro-agu/build-canopy.jpg)

*The commercial canopy and the custom payload together. Line routing and brake neutral are part of the integration work.*

HobbyKing recommends 1.6–2.0 kg for its stock paramotor, with heavier loading for windier conditions. My custom payload packages the avionics, brake actuators, batteries, motor, propeller guard, and landing structure.

![Saved SolidWorks preview of the micro-AGU payload assembly](assets/micro-agu/assembly.png)

*The payload assembly in SolidWorks.*

The assembly divides the hardware into recognizable subassemblies. A central core contains the avionics and paired servo mechanisms. A separate battery enclosure sits below it. The spreader structure, motor and shroud, and skid complete the package. That arrangement makes the aircraft's mechanical layout part of the control problem: thrust, suspension forces, and the mass of the lower payload act at different places.

![Six saved CAD previews: avionics, spreader, battery enclosure, shroud, skid, and motor](assets/micro-agu/cad-subassemblies.png)

*Avionics, spreader, battery enclosure, shroud, skid, and motor subassemblies.*

The component tree fills in details that are difficult to see in the overall preview:

| Subsystem | What the saved design contains |
|---|---|
| Brake actuation | Two instances of `ServoAssembly`, each containing a `DS339HV`, a servo horn, and a control arm |
| Avionics | `AutoPilot`, `USBBoard`, and `ESC` components within the core assembly; an `M10Q-5883` component in the main assembly |
| Propulsion | A `3115Motor` assembly, separate rotor and stator models, and an `APC 9x6 CW` propeller model |
| Battery packaging | Two instances of `Nav6S300Lion` in `BatteryBoxAssm`, plus two battery gates in the main assembly |
| Structure | Spreader assembly, core block, top block, battery enclosure, propeller shroud, and skid |

The CAD and build photographs show the configuration flown at Trout Lake. The avionics use a Matek H743-WLITE flight controller, and the two 6S Li-ion packs are connected in parallel for 6 Ah total capacity.

![The compact core held in one hand, with motor and side-mounted brake mechanisms](assets/micro-agu/build-core.jpg)

*The core before the surrounding frame and propeller guard are fitted; a useful sense of scale.*

![Open core showing the flight electronics, microSD card, wiring and power connections](assets/micro-agu/build-electronics.jpg)

*Inside the core: flight controller, power electronics, and brake-servo wiring.*

## Avionics and power

The Matek H743-WLITE runs ArduPlane, taking navigation input from the M10Q-5883 GNSS/compass and pilot commands through a RadioMaster RP1 configured in MAVLink mode. The bidirectional receiver link also carries telemetry. It commands the motor through the ESC and drives the two brake servos independently. The onboard IMU and barometer provide motion and altitude measurements; the microSD card records the flight logs used throughout this series.

![Micro-AGU avionics overview showing battery power, Matek H743-WLITE flight controller, GNSS and compass, RadioMaster RP1 in MAVLink mode, ESC, motor and two brake servos](assets/micro-agu/avionics.png)

*Orange lines show power; blue lines show control and navigation signals. This is a functional overview, with power distribution and regulation inside the H743-WLITE board boundary; connector-level wiring is omitted. [Open the vector diagram](assets/micro-agu/avionics.svg).*

The parallel battery packs feed the integrated power distribution on the H743-WLITE, which supplies the ESC and the regulated avionics and servo power. The 50 A BLHeli ESC drives the FFTY 3115 900 KV motor; each DS339HV servo rotates a brake capstan. Independent left and right brake commands let the controller steer through differential pull.

Power integrity became part of the flight-test story too. At Hood River, I was missing an electrolytic capacitor at the Matek stack's power input. The resulting erroneous current readings triggered false throttle power limiting. I corrected that before Trout Lake.

## A battery I already owned, and propulsion that fit

I already had a 6S Li-ion pack bought from GetFPV for an earlier project, so I built around it. When the aircraft needed more weight, I bought a second pack and connected them in parallel: 6S2P at the pack level, keeping the voltage and adding capacity. That let some of the required mass carry useful energy, although the later lead ballast was still necessary.

The motor was an inexpensive Amazon purchase with a KV close enough for the job. Propeller diameter was a hard constraint imposed by the payload design and my desire to keep it packable. The flown build uses an FFTY 3115 900 KV motor and a 9×4.5 propeller. The saved CAD contains an APC 9×6 CW model; that propeller designation differs from the actual flight hardware. I chose a propulsion system that fit the project rather than designing the whole aircraft around maximum endurance.

The two packs provide **6 Ah total at 6S**, roughly 130 Wh of nominal energy. At Trout Lake they powered a continuous **54.5-minute flight**, using about **85 Wh**. I landed because I had finished testing, with capacity still available. The [flight post](flight-testing.md#what-endurance-did-it-actually-achieve) breaks down the results.

## Two brake channels, and an airplane autopilot to adapt

The December 30, 2025 implementation checklist starts with the mixer: roll command to asymmetric brake output, pitch command to symmetric brake output. It also calls for a TECS patch with a gyro damper, simulation parameters, manual simulation checks, flight parameters, and trim checks.

![Original December 30 implementation and flight-test checklist](assets/micro-agu/implementation-notes.png)

*The December 30 implementation and flight-test checklist.*

That checklist captures the adaptation I was trying to make. ArduPilot supplied an existing navigation and flight-control framework, but the parafoil's controls needed different interpretation. The brake mixer was one piece. The relation between throttle and payload pitch was another, and eventually the source of a much less intuitive problem.

The [V2 manual](https://manuals.plus/m/cdab9dabe4a9759fe1f47d8eb9e2d012e56b482351e1acf6d5046c899579cf24) describes one brake for turning and both brakes for the up-elevator command. [JohnVHRC demonstrates the mixing](https://www.youtube.com/watch?v=OqNvJ_UtPqc&t=480s) and line setup. I needed to establish the neutral position and travel for my own brake mechanisms.

The same page lays out a progression through inflation and glide tests, range and failsafe checks, manual flight, FBWA/FBWB, and then LOITER and AUTO. Launch, pattern work, climb/glide, and turns were all on the list. The notebook's January 30 control-law sketches return to the lateral loop, comparing the standard ArduPlane structure with the paraglider approach. These notes connect the physical build to the later [turn-control and simulation work](paraglider.md).

My notebook also collects research on small paramotor guidance, longitudinal dynamics, and autonomous flight alongside my own sketches and test notes.

## Design choices the field would test

The prop guard and skid needed to survive awkward launches and landings. The brake mechanisms needed usable travel and a repeatable neutral position. The electronics needed reliable power and measurements. The [flight story](flight-testing.md) follows how those requirements became concrete at Barrett Park and Collin’s house.

## Ballast belongs in the mechanical model

The ballast mounts at the bottom of the suspended payload. It increases wing loading, lowers the payload CG, and changes pitch inertia. Some of the necessary mass became useful battery capacity; additional lead supplied the rest. The simulation needs to represent that mass where it actually sits.

That layout also places the thrust line above the payload CG while remaining below the suspension. Its effect on pitch became central to the [longitudinal-controller investigation](longitudinal-observer.md). Loaded mass, CG, inertia, suspension geometry, and brake settings are the measurements needed to connect the physical build to that model.

## What better performance will mean

The motivation remains the same as when I started: see what modern open-source avionics can do for a small guided parafoil. The early results establish that this powered platform can fly sustained autonomous missions. They also show why a working mission is only the beginning of the evaluation.

A useful comparison needs a recorded configuration and repeatable conditions. Guidance accuracy, wind handling, trim, and the canopy–payload motion all belong in that evaluation. For now, the design, flight data, and [longitudinal-controller investigation](longitudinal-observer.md) give me a way to identify what to measure and what to improve next.

The [data and methods appendix](data-and-methods.md#design-record) links the design evidence, source notebook pages, and CAD extraction method.

## CAD source

The [complete native SolidWorks design](../cad/README.md) is now versioned alongside the simulator, viewer and research in the Micro-AGU repository. Start with `cad/solidworks/microAGU.SLDASM`; the neighboring files contain the payload parts and subassemblies.
