# Findings

The fixed published LQI gains were tested through a 25 deg/s command. The 25 deg/s tracking gate failed, so the conditional extension above 25 was not run. Public firmware remains unchanged.

| Controller | Requested magnitude deg/s | Right actual deg/s | Left actual deg/s | Nominal cases passing |
|---|---:|---:|---:|---:|
| lqi | 10 | 10.93 | -9.89 | 2/2 |
| lqi | 12 | 13.63 | -11.92 | 2/2 |
| lqi | 13 | 15.00 | -12.96 | 0/2 |
| lqi | 15 | 17.73 | -15.06 | 0/2 |
| lqi | 20 | 24.63 | -20.26 | 0/2 |
| lqi | 25 | 31.57 | -25.69 | 0/2 |
| baseline | 15 | 15.13 | -15.17 | 2/2 |
| baseline | 25 | 25.27 | -24.93 | 2/2 |

The calm-air 12 deg/s level passes the declared absolute tolerance. The 13 deg/s level narrowly crosses the 2 deg/s mean-error threshold; this is not a physical instability boundary. At 15–25 deg/s the right-turn bias grows while altitude, airspeed and recovery remain bounded. No qualified broad-envelope increase is claimed.

The original eight-case 10 deg/s screen includes six simultaneous model/weather perturbations. Absolute rate/motion limits fail in turbulent cases at the existing limit as well. These failures include forced motion; they do not prove self-excited instability. No higher-rate held-out qualification was started after the screening failures.

## Allocation mechanism

At 15 deg/s in a steady right turn, the common-brake request is approximately −4.6% and the applied value is zero. The implemented inverse integral-gain anti-windup maps this residual into the turn integrator. Its correction is approximately −0.28 while the normalized turn error is +0.27: the turn error is nearly cancelled rather than integrated away. The speed integrator sits at its +50 bound. This is a direct arithmetic explanation of why integral action permits nonzero turn error here; it is not a separate intervention proving that changing anti-windup alone fixes the complete system. See `allocation-diagnosis.json`.

Recommended next work: make turn/speed reference equilibria feasible under nonnegative common brake, and evaluate an allocator-aware anti-windup design that preserves turn tracking when common brake clips. Retest with the same paired protocol before raising the public cap.

All results are simulation evidence for the as-built model, not a flight-tested envelope.

---

# Fixed-gain LQI turn envelope experiment

AI-assisted. Same published gains, observer, trim and sensing; only turn-reference maximum varies. CRUISE uses full stick, 40 deg/s² reference slew, sustained turns, direct reversals and stick-release recovery.

The broad-envelope screen and calm-air exploration have separate protocols. A calm-air pass is not a robust-envelope qualification. Turbulent RMS motion includes forced response and does not by itself establish self-excited instability.

| Cohort | Controller | Limit deg/s | Case | Pass | Achieved steady rates deg/s | Peak altitude error m | Reasons |
|---|---|---:|---:|---|---|---:|---|
| screen | lqi | 10 | 0 | True | 10.9, -9.9, 10.9 | 0.33 |  |
| screen | lqi | 10 | 1 | True | -9.9, 10.9, -9.9 | 0.23 |  |
| screen | lqi | 10 | 2 | False | 11.4, -10.0, 11.1 | 0.92 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| screen | lqi | 10 | 3 | False | -9.5, 11.4, -11.2 | 1.99 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| screen | lqi | 10 | 4 | False | 11.2, -9.9, 11.3 | 1.93 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| screen | lqi | 10 | 5 | False | -9.0, 10.8, -9.8 | 0.92 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| screen | lqi | 10 | 6 | False | 11.2, -9.4, 11.0 | 1.94 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| screen | lqi | 10 | 7 | False | -9.7, 10.5, -9.1 | 2.25 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| nominal | lqi | 12 | 0 | True | 13.6, -11.9, 13.6 | 0.35 |  |
| nominal | lqi | 12 | 1 | True | -12.0, 13.6, -11.9 | 0.27 |  |
| nominal | lqi | 13 | 0 | False | 15.0, -12.9, 15.0 | 0.24 | turn2: mean turn tracking |
| nominal | lqi | 13 | 1 | False | -13.0, 15.0, -13.0 | 0.29 | reverse: mean turn tracking |
| nominal | lqi | 15 | 0 | False | 17.7, -15.0, 17.7 | 0.41 | turn1: mean turn tracking; turn2: mean turn tracking |
| nominal | lqi | 15 | 1 | False | -15.1, 17.8, -15.1 | 0.45 | reverse: mean turn tracking |
| diagnostic | lqi | 20 | 0 | False | 24.6, -20.4, 24.6 | 0.53 | turn1: mean turn tracking; turn1: p90 turn tracking; turn2: mean turn tracking; turn2: p90 turn tracking |
| diagnostic | lqi | 20 | 1 | False | -20.0, 24.7, -20.4 | 0.59 | reverse: mean turn tracking; reverse: p90 turn tracking |
| diagnostic | lqi | 25 | 0 | False | 31.4, -25.9, 31.5 | 0.67 | turn1: mean turn tracking; turn1: p90 turn tracking; turn2: mean turn tracking; turn2: p90 turn tracking |
| diagnostic | lqi | 25 | 1 | False | -25.5, 31.8, -25.7 | 0.76 | reverse: mean turn tracking; reverse: p90 turn tracking |
| comparison | baseline | 15 | 0 | True | 15.2, -15.2, 14.9 | 0.43 |  |
| comparison | baseline | 15 | 1 | True | -15.3, 15.3, -15.0 | 0.39 |  |
| comparison | baseline | 15 | 2 | False | 15.4, -15.3, 15.1 | 0.95 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| comparison | baseline | 15 | 3 | False | -15.2, 15.3, -15.3 | 1.93 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| comparison | baseline | 25 | 0 | True | 25.1, -25.0, 25.6 | 0.90 |  |
| comparison | baseline | 25 | 1 | True | -25.1, 25.1, -24.7 | 0.78 |  |
| comparison | baseline | 25 | 2 | False | 25.1, -25.1, 25.2 | 1.14 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
| comparison | baseline | 25 | 3 | False | -25.1, 25.0, -25.4 | 1.83 | turn1: p90 turn tracking; turn1: turn oscillation; turn1: relative pitch peak; turn1: relative pitch late RMS; reverse: p90 turn tracking; reverse: turn oscillation; reverse: relative pitch peak; reverse: relative pitch late RMS; turn2: p90 turn tracking; turn2: turn oscillation; turn2: relative pitch peak; turn2: relative pitch late RMS; recover1: heading recovery; recover1: oscillation recovery; recover2: heading recovery; recover2: oscillation recovery |
