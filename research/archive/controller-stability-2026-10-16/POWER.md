# Power and energy comparison

These logs contain commands, actual brake travel, state and wind, but **no battery
current or retained rotor RPM**. Watts/Wh below are quasi-steady propeller shaft
power estimates from the existing 22.2 V throttle-to-RPM map and Cp table, using
logged body-axis airspeed. Source hashes match the campaign firmware manifest.

They are **not electrical power, battery energy or endurance predictions**.
Motor/ESC losses, servo power and avionics power are not modeled. The motor's
17.56 ms lag is omitted because commands were saved only every 100 ms.

## Interpretation

LQI is not consistently more energy-efficient in this campaign. In every AUTO
pair, combined-window estimated shaft energy is higher; the median paired power
increase is 11.5%, and estimated energy per traveled ground distance rises 18.4%.
LQI uses substantially more common brake and flies more slowly in these runs.
That is consistent with extra brake drag, but is not a controlled causal ablation.

CRUISE is mixed: combined-window median paired power change is approximately zero
(-0.3%), with individual changes from -19.0% to +35.8%. In the two sustained
baseline-oscillation cases (01, 15), late-calm LQI shaft-power estimates are
14–16% lower. Conversely, cases 03, 07 and 09 settle under LQI with roughly
26–27% common brake and 3.87–4.00 m/s airspeed; they require 13–17% more shaft
power than their baseline partners. Suppressing oscillation helps, but an inefficient
brake/throttle equilibrium can erase that advantage.

Windows: disturbed 20–100 s, late calm 150–180 s, combined 20–180 s after
controller selection. Combined energy ratios closely follow mean-power ratios
because the paired windows have the same duration. Settling and launch are excluded.

All values are time-weighted. Group rows are medians across eight cases.

| Window | Mode | Controller | Throttle % | Est. shaft W | Est. shaft Wh/km ground path | Ground speed m/s | Common brake % |
|---|---|---|---:|---:|---:|---:|---:|
| disturbed | AUTO | baseline | 40.08 | 58.70 | 3.76 | 4.23 | 4.56 |
| disturbed | AUTO | lqi | 43.24 | 70.24 | 4.91 | 3.97 | 21.79 |
| disturbed | CRUISE | baseline | 39.87 | 58.28 | 3.38 | 4.56 | 2.76 |
| disturbed | CRUISE | lqi | 40.02 | 60.47 | 3.85 | 4.53 | 10.32 |
| late_calm | AUTO | baseline | 44.58 | 73.64 | 4.67 | 4.38 | 2.93 |
| late_calm | AUTO | lqi | 46.22 | 81.32 | 5.59 | 4.04 | 22.92 |
| late_calm | CRUISE | baseline | 44.64 | 73.90 | 4.68 | 4.38 | 2.95 |
| late_calm | CRUISE | lqi | 44.66 | 73.98 | 4.72 | 4.36 | 4.17 |
| combined | AUTO | baseline | 42.49 | 66.91 | 4.29 | 4.30 | 3.76 |
| combined | AUTO | lqi | 44.51 | 74.73 | 5.15 | 4.08 | 19.48 |
| combined | CRUISE | baseline | 42.39 | 66.29 | 4.19 | 4.47 | 2.83 |
| combined | CRUISE | lqi | 43.05 | 69.99 | 4.46 | 4.39 | 11.53 |

## Paired relative changes: LQI vs baseline

Ratios are calculated within each pair before taking the median and range.

- disturbed AUTO, estimated_shaft_W: +20.7% (range +15.1% to +32.7%).
- disturbed AUTO, estimated_shaft_Wh_per_ground_km: +29.4% (range +15.3% to +36.7%).
- disturbed CRUISE, estimated_shaft_W: +7.7% (range -21.2% to +73.4%).
- disturbed CRUISE, estimated_shaft_Wh_per_ground_km: +13.3% (range -14.1% to +45.8%).
- late_calm AUTO, estimated_shaft_W: +10.4% (range -1.3% to +14.4%).
- late_calm AUTO, estimated_shaft_Wh_per_ground_km: +19.7% (range -1.3% to +28.2%).
- late_calm CRUISE, estimated_shaft_W: -0.2% (range -16.3% to +16.9%).
- late_calm CRUISE, estimated_shaft_Wh_per_ground_km: -0.2% (range -20.3% to +32.4%).
- combined AUTO, estimated_shaft_W: +11.5% (range +7.0% to +19.3%).
- combined AUTO, estimated_shaft_Wh_per_ground_km: +18.4% (range +9.1% to +26.2%).
- combined CRUISE, estimated_shaft_W: -0.3% (range -19.0% to +35.8%).
- combined CRUISE, estimated_shaft_Wh_per_ground_km: +2.1% (range -17.5% to +31.3%).

## CRUISE late-calm cases

| Case | Controller | Est. shaft W | Est. Wh/km ground path | Height change m | Speed change m/s |
|---|---|---:|---:|---:|---:|
| 1 | baseline | 87.82 | 5.85 | -0.051 | 0.528 |
| 1 | lqi | 73.53 | 4.66 | -0.053 | -0.001 |
| 3 | baseline | 73.75 | 4.67 | 0.037 | -0.002 |
| 3 | lqi | 85.40 | 6.04 | -0.046 | 0.001 |
| 5 | baseline | 73.81 | 4.68 | 0.060 | 0.002 |
| 5 | lqi | 73.55 | 4.66 | -0.046 | -0.002 |
| 7 | baseline | 73.91 | 4.68 | 0.100 | -0.001 |
| 7 | lqi | 83.28 | 5.79 | -0.035 | 0.001 |
| 9 | baseline | 74.12 | 4.70 | 0.185 | 0.006 |
| 9 | lqi | 86.62 | 6.22 | -0.041 | -0.002 |
| 11 | baseline | 73.89 | 4.68 | 0.094 | -0.002 |
| 11 | lqi | 73.53 | 4.66 | -0.051 | -0.002 |
| 13 | baseline | 73.62 | 4.67 | -0.016 | -0.002 |
| 13 | lqi | 73.55 | 4.66 | -0.047 | -0.000 |
| 15 | baseline | 86.97 | 5.80 | 0.040 | 0.024 |
| 15 | lqi | 74.41 | 4.77 | -0.051 | 0.090 |

Ground-path distance includes all traveled distance, not useful mission progress.
AUTO paths and target tracking differ between controllers. Disturbed flight also
has unequal thermal encounters and changes in stored potential/kinetic energy.
Late calm CRUISE is the cleaner comparison, but finite windows and oscillation
phase still affect endpoint energy. No result here establishes electrical efficiency.

Reproduce with a NumPy/SciPy environment: `python power.py`. No reruns or retuning.
`test_power.py` verifies the reconstructed shaft-power calculation against the
native Rust torque map across 42 throttle/axial-speed combinations; it passes.
This verifies interpolation and units, not the omitted motor dynamics or electrical losses.
