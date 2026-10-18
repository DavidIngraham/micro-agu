#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
ap=/home/david/repos/ardupilot-browser-9dof
out="$root/research/archive/filtered-brake-2026-10-16/validation"
export PATH="/home/david/envs/venv-ardupilot/bin:/usr/local/bin:/usr/bin:/bin"
cd "$root"
simulation/.venv/bin/python research/archive/legacy-control/design_mimo.py --output simulation/data/control/mimo > "$out/final-design.log"
cp simulation/data/control/mimo/AP_CustomControl_LQI_gains.h "$ap/libraries/AP_CustomControl/"
cd "$ap"
./waf configure --board sitl > "$out/native-build.log" 2>&1
./waf plane --targets tests/test_mimo -j4 >> "$out/native-build.log" 2>&1
build/sitl/tests/test_mimo > "$out/native-unit.log" 2>&1
Tools/autotest/autotest.py test.Plane.ParagliderMIMO > "$out/native-autotest.log" 2>&1
