#!/usr/bin/env bash
# Compatibility helper: normal vehicle startup prepares this automatically.
set -euo pipefail
tool_dir="$(dirname -- "${BASH_SOURCE[0]}")"
python3 "$tool_dir/prepare_brake_vehicle_runtime.py"
