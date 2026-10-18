# LQI development explanation

The brake diamond is a normalized geometric demonstration: left = common + differential; right = common - differential, each constrained to [0, 1]. It deliberately does not emulate the deployed allocator, dynamics or gain matrix. No simulated performance is inferred from moving its controls.

The eight comparison rows are copied without rounding from `viewer/assets/runtime/lqi-longitudinal-validation.json`, `summary.pairs`. Regenerate evidence.json from that field when updating the release. All metrics share the same cases. Displayed ratios retain failed baseline recovery context. The article explains the separate hard screens.

Both controls use the shared applet lifecycle; all event listeners are disposed on unmount. No autonomous animation (including under reduced motion).
