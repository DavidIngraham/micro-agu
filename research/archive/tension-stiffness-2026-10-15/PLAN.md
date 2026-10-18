# Frozen experiment plan

Test whether load-dependent geometric stiffness improves agreement across operating conditions without controller retuning. This is a reduced-model sensitivity study, not a distributed, tension-only line model. Retain the current point constraint, fixed damping, aerodynamic coefficients, propulsion, mass and geometry.

Variants: fixed; yaw-only load dependence; roll-only; both; half of both stiffnesses geometric. Define the geometric scale as clamp(T/T0, 0.25, 2), with T the tensile component of the solved joint reaction along the canopy-to-attachment direction. T0 is evaluated at each flight configuration's existing 22.2 V straight-level trim. Spring torque and joint reaction are solved simultaneously using their affine dependence on this single scale. Damping is not scaled. At T=T0, all variants have the same nominal spring coefficients and trim torque, though local derivatives may differ. Clipping is a regularization, not a model of slack. Record clipping, compressive loads and implicit-solve conditioning.

Before results: verify fixed-model equivalence, retained trim, simultaneous-load residual, joint acceleration constraint and integration refinement. Reject a singular/negative feedback denominator rather than hide it.

Flight comparison: replay 30 s from Hood River starts [300,360,480,600,690,720,930,1020] and Trout Lake [600,840,900,1350,1920,2400,2790,2910]. These existing flights informed the model; this is not independent flight validation. Same measured payload state, recorded commands, initial RPM, wind assumption and initial voltage for every variant. Use the prior force-based canopy initialization with 0.35 rad angle of attack; repeat with 0.25 and 0.45 rad to expose hidden-state sensitivity. Do not fit initial states or model parameters per variant. Assess roll/pitch/yaw-rate RMSE, bank error, course error and climb error; summarize equally per window. Prioritize reducing roll/yaw-rate errors without >5% aggregate pitch, bank, course or climb regression. Preserve all failures.

Closed-loop regression: the eight scheduled-study scenarios, unchanged conventional baseline gains, same reference/actuator handling, truth sensing. Compare fixed versus all four sensitivity variants. This checks behavior with the existing controller, not agreement with flight or controller optimality. Repeat representative high-load cases at twice the integration resolution.

No deployment, flight-controller retuning or claim of full line mechanics is part of this experiment.
