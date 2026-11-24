#!/usr/bin/env bash
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$project/native" -B "$project/native/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$project/native/build" -j2
