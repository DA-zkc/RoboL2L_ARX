#!/usr/bin/env bash
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export LD_LIBRARY_PATH="$project/vendor/realsense/lib:$project/vendor/orbbec/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$project/.venv/bin/python" -m franka_player --port 8768 --data-dir "$project/.franka-player" --cap-station "$project/config/station.json" --runtime-factory franka_cap.runtime:create_runtime "$@"
