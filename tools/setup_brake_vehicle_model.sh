#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -x "$root/.venv/bin/python" ]]; then
  py="$root/.venv/bin/python"
else
  py="$(command -v python3)"
fi
command -v uv >/dev/null || { echo "uv is required to install the optional ONNX runtime."; exit 1; }
uv pip install --python "$py" --only-binary=:all: 'onnxruntime==1.22.1'
"$py" -c 'import onnxruntime; print("Brake vehicle runtime ready:", onnxruntime.get_available_providers())'
echo "Restart the UI to enable the optional detector. LEAD_BRAKE_MODEL=0 disables it."
