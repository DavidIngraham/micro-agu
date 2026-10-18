# Sensing comparison

Power is modeled propeller shaft power from retained RPM, not battery power.

| Mode | Variant | Complete | Pitch RMS ° | Alt RMS m | Xtrack RMS m | Late pitch RMS ° | Shaft W | Common brake % |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| AUTO | airspeed | 8/8 | 3.933 | 3.870 | 4.603 | 0.037 | 70.942 | 3.471 |
| AUTO | angles | 2/2 | 3.711 | 3.666 | 10.930 | 0.029 | 69.316 | 3.503 |
| AUTO | full | 2/2 | 3.209 | 5.193 | 19.622 | 3.020 | 162.935 | 95.073 |
| AUTO | sensor | 8/8 | 3.893 | 3.888 | 4.580 | 0.031 | 70.965 | 3.507 |
| AUTO | sensorless | 8/8 | 4.063 | 3.874 | 4.686 | 0.031 | 76.046 | 13.659 |
| AUTO | standard | 8/8 | 3.930 | 1.845 | 4.089 | 0.035 | 66.746 | 0.000 |
| CRUISE | airspeed | 8/8 | 3.940 | 1.645 | 0.406 | 0.033 | 68.109 | 2.357 |
| CRUISE | angles | 2/2 | 3.752 | 2.259 | 0.229 | 0.026 | 67.570 | 1.524 |
| CRUISE | full | 2/2 | 2.786 | 11.868 | 34.561 | 2.546 | 163.115 | 95.717 |
| CRUISE | sensor | 8/8 | 3.946 | 1.662 | 0.409 | 0.034 | 68.064 | 2.354 |
| CRUISE | sensorless | 8/8 | 4.017 | 1.403 | 0.527 | 0.033 | 76.314 | 18.278 |
| CRUISE | standard | 8/8 | 3.823 | 2.698 | 0.241 | 0.036 | 65.970 | 0.000 |

## Sensor fidelity

| Variant | Canopy angle error ° | Canopy rate error °/s | Airspeed error m/s | Airspeed use |
|---|---:|---:|---:|---:|
| airspeed | 0.764 | 2.922 | 0.173 | 1.000 |
| angles | 0.114 | 9.538 | 0.175 | 1.000 |
| full | 0.118 | 1.293 | 0.183 | 1.000 |
| sensor | 0.732 | 3.415 | 0.175 | 1.000 |
| sensorless | 0.696 | 3.317 | 0.776 | 0.000 |

## Failures

