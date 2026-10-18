#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
ap=/home/david/repos/ardupilot-browser-9dof
name=${1:?name}
weight=${2:?weight}
destination="$root/research/archive/airspeed-sensor-2026-10-16/$name"
mkdir -p "$destination"
export PATH="/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:$PATH"
cd "$root"
simulation/.venv/bin/python research/archive/legacy-control/design_mimo.py --output "$destination" --brake-memory-weight "$weight" --diagnostics > "$destination/design.log"
cp "$destination/AP_CustomControl_LQI_gains.h" "$ap/libraries/AP_CustomControl/"
cp research/archive/legacy-control/design_mimo.py "$destination/design_mimo.py.txt"
cp "$ap/libraries/AP_CustomControl/AP_CustomControl_LQI.cpp" "$destination/backend.cpp.txt"
cp viewer/packages/sim-browser/src/physics/physics.ts "$destination/physics.ts.txt"
cp viewer/packages/sim-browser/src/physics/airspeed.ts "$destination/airspeed.ts.txt"
bash viewer/tools/build/build_sitl_wasm.sh "$ap" > "$destination/build.log" 2>&1
python research/archive/airspeed-sensor-2026-10-16/run.py "$name" "${3:-0,1,6,9}"
