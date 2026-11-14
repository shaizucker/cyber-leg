#!/usr/bin/env bash
set -euo pipefail
bridge_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
bridge_python="${BRIDGE_PYTHON:-$bridge_root/.venv/bin/python}"
if [[ ! -x "$bridge_python" ]]; then
  echo "Create the virtual environment first; see docs/Jetson/setup.md" >&2
  exit 1
fi
exec "$bridge_python" -u "$bridge_root/drivers/knee_bridge.py" \
  --can-interface socketcan --can-channel "${CAN_CHANNEL:-can1}" \
  --ankle-port "${ANKLE_PORT:-/dev/ttyACM0}" "$@"
