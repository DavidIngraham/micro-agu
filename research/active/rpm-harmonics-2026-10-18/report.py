import json,hashlib,subprocess
from pathlib import Path
import numpy as np
OUT=Path(__file__).resolve().parent
a=json.loads((OUT/"audit.json").read_text());r=json.loads((OUT/"windows.json").read_text())
lines=["# ESC RPM harmonic and alias audit","",
"AI-assisted analysis, 2026-10-18. Existing Hood River and Trout Lake logs only. No simulator, gains, filter settings or deployment changed.","",
"## Finding","",
"ESC RPM is available in both flights and was already used by prior propulsion/model studies. The dominant low-frequency yaw component does not consistently track shaft-speed or two-blade blade-pass aliases. That argues against a simple rotating-order alias explaining the coupled oscillation, but cannot exclude high-frequency vibration, nonlinear demodulation, or local flight-controller mounting motion.","",
"## Available measurements","",
"| Flight | ESC records | ESC rate | Gyro log rate | Accepted 20 s windows |",
"|---|---:|---:|---:|---:|"]
for f,x in a.items():
 lines.append(f"| {f} | {x['esc_messages']} | {1/x['esc_median_dt']:.2f} Hz | ~50 Hz | {x['windows']} |")
lines+=["",
"Both original binary logs were inventoried, not merely the extracted JSON: ACC, GYR, ISBH and ISBD have zero records. IMU instance 0 is the only extracted IMU. Batch logging was disabled (INS_LOG_BAT_MASK=0); both harmonic notches were disabled and INS_GYRO_FILTER=20 Hz. INS_GYRO_RATE=1 and the logged gyro-rate counter around 2025 Hz indicate a much faster internal path than the 50 Hz saved stream, but do not establish every internal sampling/filtering stage.",
"",
"The historical Hood River firmware AP_BLHeli.cpp converts ESC electrical speed by 200 / motor_poles before update_rpm. SERVO_BLH_POLES=14 in both logs. AP_ESC_Telem get_rpm slews between updates; RawRPM returns the unslewed stored value. Thus these fields are treated as mechanical RPM under the configured pole count, not divided by seven again. An old logger comment calling these eRPM is inconsistent with this backend conversion. No independent tachometer validates the pole-count setting here.",
"",
"RPM is sampled around every 0.12 s: adequate for a speed track, inadequate for precise shaft-phase reconstruction at 100+ revolutions/s. RPM and RawRPM both enter alias checks. RPM-derived high-order frequency errors grow with order, and quantized telemetry can create apparently neat coincidences.",
"",
"## Method","",
"Use nonoverlapping 20 s windows with at least 95% validity under the existing flight-data mask, median RPM above 1500, at least 100 ESC samples and 900 gyro samples, no ESC gap above 0.3 s or gyro gap above 0.06 s. These gates select powered flight, not stationary-RPM tests. Median RPM ranges must not be interpreted as steady operating points; several windows contain large speed changes.",
"",
"Interpolate logged gyro and accelerometer channels onto a 50 Hz grid. Welch spectra use 10 s subwindows and 50% overlap (0.1 Hz bin spacing). Pick the strongest yaw peak over 0.8–5 Hz, independently of the target 1.2–1.9 Hz band. Integrate band power and compare to RPM and VIBE. Descriptive Spearman correlations are confounded by flight condition, input and mode; no causal significance is claimed.",
"",
"Rotating orders 1, 2, 3, 4, 7, 14 and 42 were screened. Order 2 represents blade passage for the two-blade propeller; 7 is the electrical fundamental for 14 poles, while 14 and 42 are exploratory electrical/commutation-related orders, not established mechanical forcing lines. Order frequencies and nominal logging aliases are",
"",
"$$",
r"f_n=n\,\mathrm{RPM}/60,\qquad f_a=\left|((f_n+f_s/2)\bmod f_s)-f_s/2\right|.",
"$$",
"",
"This models folding into a nominal 50 Hz saved stream, not the full undocumented sensor-to-logger transfer function. We also check 49.5 and 50.5 Hz as deliberately broad sensitivity cases; measured average log rates in accepted windows are essentially 50 Hz.",
"",
"A coincidence means predicted alias within 0.2 Hz of the observed yaw peak. We evaluate both median-RPM predictions and each ESC sample within its window. Circularly shifted window RPM tracks provide a descriptive chance-alignment comparison. Multiple harmonics and correlated windows prevent treating isolated matches as identification.",
"",
"## Results","",
"| Flight | Windows with peak in 1.2–1.9 Hz | RPM vs peak frequency | RPM vs band amplitude | VIBE magnitude vs band amplitude |",
"|---|---:|---:|---:|---:|"]
for f,x in a.items():
 z=[v for v in r if v["flight"]==f]
 fraction=np.mean([1.2-1e-9<=v["yaw_peak_hz"]<=1.9+1e-9 for v in z])
 lines.append(f"| {f} | {fraction:.1%} | {x['rpm_peak_spearman']:.2f} | {x['rpm_band_amplitude_spearman']:.2f} | {x['vibe_band_amplitude_spearman']:.2f} |")
lines+=["",
"The last three columns are Spearman correlations. The peak tends to move slightly downward with RPM rather than rising proportionally to shaft speed. This is compatible with a changing coupled mode but is not proof of gyroscopic causation. Amplitude has only a modest positive association with RPM and VIBE.",
"",
"| Flight | 1x alias within-window coincidence | 2x alias within-window coincidence |",
"|---|---:|---:|"]
for f,x in a.items():
 lines.append(f"| {f} | {x['alias_null'][0]['within_window_coincidence']:.1%} | {x['alias_null'][1]['within_window_coincidence']:.1%} |")
lines+=["",
"These are fractions of ESC sample times matching the window peak, averaged across windows, not probabilities that vibration is causal. Repeating with RawRPM gives approximately the same low coincidence. Median-RPM matches are sparse and comparable to shifted-track coincidences; sample-rate sensitivity does not produce a convincing match.",
"",
"![RPM versus observed yaw peak](rpm-peak.png)",
"",
"![Hood River RPM and gyro spectrogram](hood_river-spectrogram.png)",
"",
"![Trout Lake RPM and gyro spectrogram](trout_lake-spectrogram.png)",
"",
"The spectrogram colors show yaw-rate power, with fixed display range -20 to 30 dB relative to (deg/s)²/Hz. Cyan and green dots are predicted 1x and 2x aliases, not measured spectral peaks. Ten-second spectral windows smear rapid speed changes, so individual dot intersections are not a test of phase coherence. The persistent low-frequency ridge generally remains while predicted aliases move through it.",
"",
"No VIBE clipping increments occurred in the 199 accepted windows. Hood River has 29 increments elsewhere in the full log; Trout Lake has none. VIBE clipping concerns accelerometers, so this does not rule out gyro vibration or local board motion.",
"",
"## What this does and does not establish","",
"- The saved records contain real ESC RPM; missing RPM was not the limitation.",
"- Typical 6000–9000 RPM corresponds to 100–150 Hz shaft frequency and 200–300 Hz two-blade passage, well above the saved gyro stream's 25 Hz Nyquist limit.",
"- Simple 1x/2x folding into the saved stream is not a convincing explanation for the persistent ~1.6 Hz component.",
"- Higher-order coincidences are especially sensitive to RPM quantization, delay, pole scaling and the actual internal sampling path. This is not a definitive high-frequency vibration-source identification.",
"- A motor can excite a low-frequency structural/aircraft resonance without that response occurring at shaft frequency. Nonlinear envelope modulation, local mounting motion and physical gyroscopic coupling remain possible.",
"- Short-window RPM/yaw coherence is retained as a supplementary diagnostic only. It concerns speed modulation, not rotational phase, and cannot establish direction of causality.",
"",
"## Next useful measurement","",
"Capture high-rate pre/post-filter gyro and accelerometer data with ESC RPM during a safely controlled motor-speed sweep, then compare actual spectral ridges in shaft-order coordinates. For flight, synchronized video or a second independently mounted IMU can distinguish whole-payload rotation from local board motion. Do not add a notch at ~1.6 Hz from this evidence; that could suppress genuine flight dynamics.",
"",
"ArduPilot documents [high-rate IMU batch sampling and aliasing](https://ardupilot.org/copter/docs/common-imu-batchsampling.html) and [RPM-tracked harmonic filtering](https://ardupilot.org/copter/docs/common-imu-notch-filtering.html).",
"",
"## Reproduction","",
"Run research/.venv/bin/python research/active/rpm-harmonics-2026-10-18/analyze.py, then report.py. Set MICROAGU_FLIGHT_ANALYSIS to relocate extracted inputs. inventory.py additionally needs pymavlink and reads the original binary log paths. checks.py verifies synthetic alias folding and spectral peak recovery. Source and input hashes are retained; no raw logs are committed."]
(OUT/"README.md").write_text(chr(10).join(lines)+chr(10))
(OUT/"manifest.json").write_text(json.dumps(dict(parent_commit=subprocess.check_output(["git","rev-parse","HEAD"],text=True).strip(),source_hashes={x.name:hashlib.sha256(x.read_bytes()).hexdigest() for x in OUT.glob("*.py")},decision="No simple shaft/blade-pass alias identification; high-rate source data absent"),indent=2))
