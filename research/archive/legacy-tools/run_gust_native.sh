#!/usr/bin/env bash
# Run from an already-built ArduPilot feature checkout. Raw output must be new.
# Usage: run_gust_native.sh ARDUPILOT_REPO OUTPUT_DIRECTORY CONFIG_JSON gust|calm|crosswind
set -euo pipefail
repo=$(realpath "$1")
output=$(realpath -m "$2")
config=$(realpath "$3")
condition=$4
scripts=$(cd "$(dirname "${BASH_SOURCE[0]}")/../research/archive/viewer-studies" && pwd)
case "$condition" in gust) options=(--turbulence);; calm) options=();; crosswind) options=(--wind-ned 0 2 0);; *) echo 'Use gust, calm or crosswind' >&2; exit 2;; esac
mkdir -p "$output"
cd "$repo"
export BUILDLOGS="$output" PG_CONTROLLER_CONFIG="$config" PG_GAINS=original PG_VOLTAGE=22.2
python "$scripts/record_auto.py" "${options[@]}" --model-dir "$scripts/../../simulation" --output "$output/truth.jsonl" -- --no-check-parameter-leaks --speedup 10 test.Plane.ParagliderAutoMission > "$output/run.log" 2>&1
python "$scripts/../../viewer/tools/convert/extract_auto_clip.py" "$output"
