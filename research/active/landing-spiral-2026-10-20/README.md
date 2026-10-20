# Tangential landing spiral

The native landing example now uses:

1. One clockwise capture orbit, radius 40 m, at 60 m above home.
2. `MAV_CMD_NAV_LOITER_TO_ALT` (31) at the same centre, descending to 30 m.
3. A level approach waypoint 200 m south of home at 30 m.
4. Native `NAV_LAND` at home.

The orbit centre is 280 m south and 40 m east of home, so its west edge is
geometrically tangent to the northbound approach line. Command 31 uses param2
for the signed radius and param4=1 to track from the actual exit point, rather
than from the loiter centre. The capture orbit prevents descending on the
inbound leg from using up the intended spiral. No firmware or gains change.

The map and 3D preview show the orbit, descending spiral and tangent leg.
They depict nominal geometry, not a prediction of the number of turns or the
exact wind-dependent flight path. ArduPlane uses a 10-degree initial heading
acceptance when exiting a loiter; it can relax that tolerance after extra turns.
The measured exit headings below are consistent with that existing behavior.

## Validation

Real packaged WASM firmware, EKF and Rust physics; LQI selected. Two runs use
calm air and a 1.5 m/s crosswind with 0.2 turbulence, respectively.

| Case | Spiral exit altitude | Descending turn | Exit heading error | Final heading error | Final cross-track | Touchdown sink |
|---|---:|---:|---:|---:|---:|---:|
| Calm | 29.99 m | 272.3° | −8.45° | 1.07° | −0.06 m | 0.90 m/s |
| Crosswind | 30.14 m | 272.8° | −6.65° | 1.00° | −0.15 m | 0.87 m/s |

Both remained in AUTO through landing. Spot errors were 10.24 m and 11.39 m.
These are two simulator checks, not flight qualification or proof
of exact tangency in all winds. Geometry unit tests check perpendicular radius,
clockwise direction, command parameter mapping and the preview's exit point.
The browser test covers mission upload, cancellation/resumption and touchdown.

From the repository root:

```sh
python3 research/active/landing-spiral-2026-10-20/run.py
python3 research/active/landing-spiral-2026-10-20/assess.py
```

The runner reuses the native-landing study's firmware harness, selecting the
current fixed mission. `provenance.json` pins the tested mission and artifacts.
`evidence.tar.gz` contains the two raw traces; extract into `runs/` to reassess.
