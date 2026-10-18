# Airspeed sensor comparison

AI-assisted simulation experiment, not hardware sensor qualification.
Compare the same zero-common-trim LQI with memory weights 0 and 3000, time constant
3 s. Keep the EKF, full external physics, flight-log calibration, 132 mm geometry,
22.2 V, weather seeds, model perturbations and flight windows unchanged.

Research case overrides: ARSPD_TYPE=100 (SITL pressure sensor), ARSPD_USE=1,
ARSPD_SKIP_CAL=1, ARSPD_OFFSET=0, ARSPD_RATIO=1.99, SIM_ARSPD_RATIO=1.99,
SIM_ARSPD_RND=2. The digital SITL backend returns pressure without the simulated
analog offset; skip calibration represents known zero offset, not an in-flight
calibration. The default 2 Pa pressure noise remains enabled. No direct truth
measurement is injected into the LQI; measurements pass through the existing
sensor/AHRS interfaces. Other estimation and sensor checks retain their defaults.

First screen scenarios 0,1,6,9 (nominal AUTO/CRUISE and two diagnosed biased cases).
If the sensor is used and the runs complete, extend both weights to all 16 cases.
Compare against the retained sensorless `filtered-brake-2026-10-16` runs with the
same weights. Launch and initial estimation differ when enabling the sensor;
this is an end-to-end configuration comparison, not identical-state replay.

The trailing field in new LQID records is AHRS `using_airspeed_sensor()`. Check it
throughout active control; do not infer use from the configured parameter alone.
Truth is retained only for analysis. Report airspeed error, speed-integrator
saturation, common brake, tracking, pitch recovery and estimated shaft energy.
Energy uses the same quasi-steady calculation as prior campaigns.

No change to the published simulator or its default sensor configuration.

The initial four-case `sensor-zero` and `sensor-3000` screens exposed a transport
unit error: browser JSON supplied TAS, but `SIM_JSON.cpp` assigns that directly to
the equivalent-airspeed pitot input. AHRS subsequently applies EAS-to-TAS conversion,
producing an altitude-dependent positive bias. The corrected `eas-zero` and
`eas-3000` runs convert TAS to EAS using the same tropospheric atmosphere as AP_Baro
and the simulation home altitude plus aircraft height. The force model is unchanged.
Initial screens are retained separately and excluded from the final comparison.
The conversion has tests against ArduPilot's atmosphere reference values.
