#!/usr/bin/env bash
# Select the station backend before starting an experiment. Motion is opt-in.
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
backend="${1:-}"
case "$backend" in
    panda_py|franky) station="$project/config/station.json" ;;
    arx_x5_2023|arx_x5_2023_dual)
        station="$project/config/arx-x5-2023.json"
        if [[ "$backend" == arx_x5_2023_dual ]]; then station="$project/config/arx-x5-2023-dual.json"; fi
        export PYTHONPATH="${ARX5_SDK_DIR:-$project/../ARX5_beta}${PYTHONPATH:+:$PYTHONPATH}"
        ;;
    *) echo 'Usage: scripts/start-hardware.sh {panda_py|franky|arx_x5_2023|arx_x5_2023_dual} [server options]'; exit 0 ;;
esac
shift
cd "$project"
export LD_LIBRARY_PATH="$project/vendor/realsense/lib:$project/vendor/orbbec/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$project/.venv/bin/python" -m franka_player.server \
    --hardware-station "$station" --robot-backend "$backend" \
    --data-dir "$project/.franka-player/workbench-$backend" --port 8767 "$@"
