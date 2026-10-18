#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
study="$root/research/archive/filtered-brake-2026-10-16"
export PATH="/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:$PATH"
cd "$root"
for name in memory-3000 diagnostic-zero; do
    cp "$study/$name/arduplane.js" viewer/generated/ardupilot/arduplane.js
    cp "$study/$name/arduplane.wasm" viewer/generated/ardupilot/arduplane.wasm
    cp "$study/$name/build.json" viewer/assets/runtime/build.json
    python simulation/tools/build_metadata.py
    python research/archive/zero-brake-2026-10-16/run.py "$study/$name" --cases 2,4,7,8,10,11,12,14 >> "$study/$name/run.log" 2>&1
    python "$study/analyze.py" "$name" > "$study/$name/analysis.txt"
done
python "$study/report.py"
