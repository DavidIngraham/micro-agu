#!/usr/bin/env bash
set -euo pipefail
root=/home/david/repos/micro-agu
out="$root/research/archive/filtered-brake-2026-10-16/validation"
export PATH="/home/david/.cargo/bin:/home/david/envs/venv-ardupilot/bin:/home/david/emsdk/node/24.19.0_64bit/bin:$PATH"
cd "$root"
bash viewer/tools/build/build_sitl_wasm.sh /home/david/repos/ardupilot-browser-9dof > "$out/wasm-build.log" 2>&1
cp viewer/assets/runtime/build.json "$out/final-build.json"
cd viewer
npm run build > "$out/viewer-build.log" 2>&1
npm run test:mimo > "$out/browser-mimo-final.log" 2>&1
