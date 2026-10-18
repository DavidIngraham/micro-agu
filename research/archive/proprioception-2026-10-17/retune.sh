#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
study="$root/research/archive/proprioception-2026-10-17"
ap=/home/david/repos/ardupilot-browser-9dof
export PATH=/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:/usr/local/bin:/usr/bin:/bin
cd "$root"
restore() { simulation/.venv/bin/python "$study/prepare.py" restore; }
trap restore EXIT
for name in efficient damping; do
    simulation/.venv/bin/python "$study/install.py" "$name-design"
    bash viewer/tools/build/build_sitl_wasm.sh "$ap" > "$study/$name-build.log" 2>&1
    simulation/.venv/bin/python "$study/install.py" "$name-design" archive
    python "$study/run.py" "$name" sensor 0,1,6,9 > "$study/$name-run.log" 2>&1
    python "$study/analyze.py" "$name" "$name-design" > "$study/$name-analysis.log"
done
python "$study/select.py" > "$study/selection.log"
selected=$(cat "$study/selected.txt")
if [[ "$selected" != fixed ]]; then
    simulation/.venv/bin/python "$study/install.py" "$selected-design"
    bash viewer/tools/build/build_sitl_wasm.sh "$ap" > "$study/selected-build.log" 2>&1
    python "$study/run.py" "$selected" sensor 2,3,4,5,7,8,10,11,12,13,14,15 > "$study/selected-run.log" 2>&1
    python "$study/analyze.py" "$selected" "$selected-design" > "$study/selected-analysis.log"
fi
python "$study/audit.py" > "$study/audit.log" 2>&1
python "$study/provenance.py" > "$study/provenance.log" 2>&1
