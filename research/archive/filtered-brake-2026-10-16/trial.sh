#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
ap=/home/david/repos/ardupilot-browser-9dof
name=${1:?trial name}
weight=${2:?memory cost}
cases=${3:-0,1,3,5,6,9,13,15}
destination="$root/research/archive/filtered-brake-2026-10-16/$name"
test ! -e "$destination/build.json"
mkdir -p "$destination"
export PATH="/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:$PATH"
cd "$root"
simulation/.venv/bin/python research/archive/legacy-control/design_mimo.py --output "$destination" --brake-memory-weight "$weight" --brake-memory-tau 3 --diagnostics > "$destination/design.log"
cp "$destination/AP_CustomControl_LQI_gains.h" "$ap/libraries/AP_CustomControl/"
cp research/archive/legacy-control/design_mimo.py "$destination/design_mimo.py.txt"
cp "$ap/libraries/AP_CustomControl/AP_CustomControl_LQI.cpp" "$destination/backend.cpp.txt"
bash viewer/tools/build/build_sitl_wasm.sh "$ap" > "$destination/build.log" 2>&1
cp viewer/generated/ardupilot/arduplane.js "$destination/arduplane.js"
cp viewer/generated/ardupilot/arduplane.wasm "$destination/arduplane.wasm"
python research/archive/zero-brake-2026-10-16/run.py "$destination" --cases "$cases" > "$destination/run.log" 2>&1
