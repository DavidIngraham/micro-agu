#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
study="$root/research/archive/airspeed-sensor-2026-10-16"
export PATH="/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:$PATH"
cd "$root"
restore() {
    cp simulation/data/control/mimo/AP_CustomControl_LQI_gains.h /home/david/repos/ardupilot-browser-9dof/libraries/AP_CustomControl/
    bash viewer/tools/build/build_sitl_wasm.sh /home/david/repos/ardupilot-browser-9dof > "$study/final-build.log" 2>&1
}
trap restore EXIT
for name in eas-zero eas-3000; do
    weight=0
    if [[ "$name" == eas-3000 ]]; then weight=3000; fi
    bash "$study/trial.sh" "$name" "$weight" 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 > "$study/$name-run.log" 2>&1
    python research/archive/filtered-brake-2026-10-16/analyze.py "$study/$name" > "$study/$name/analysis.txt"
done
python "$study/report.py"
