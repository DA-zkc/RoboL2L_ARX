#!/usr/bin/env bash
# Local, pinned build. Does not connect to a robot or change system libraries.
set -euo pipefail
project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project"
franky_commit=f88f0e9b8f598d7c537b4e727e4411e67c95e017
ruckig_commit=cb99a04ce488f83701aaee6efd9c9f0d36a3d43b
for item in "franky|https://github.com/TimSchneider42/franky.git|$franky_commit" "ruckig|https://github.com/pantor/ruckig.git|$ruckig_commit"; do
    IFS='|' read -r name url revision <<< "$item"
    if [[ ! -d "vendor/$name/.git" ]]; then
        git clone "$url" "vendor/$name"
        git -C "vendor/$name" checkout --detach "$revision"
    fi
    [[ "$(git -C "vendor/$name" rev-parse HEAD)" == "$revision" ]] || { echo "Wrong $name source revision" >&2; exit 1; }
done
if ! git -C vendor/franky apply --reverse --check "$project/patches/franky-station.patch" 2>/dev/null; then
    git -C vendor/franky apply --check "$project/patches/franky-station.patch"
    git -C vendor/franky apply "$project/patches/franky-station.patch"
fi
cp native/franky_station.cpp vendor/franky/python/franky_station.cpp
cp native/sampled_joint_curve.h vendor/franky/python/sampled_joint_curve.h
if [[ ! -d .franka-player/build-tools/pybind11 ]]; then
    .venv/bin/python -m pip install --target .franka-player/build-tools pybind11==3.1.0
fi
.venv/bin/python -c 'import websockets.sync.client'
cmake -S vendor/ruckig -B native/build/ruckig -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$project/.franka-player/native" -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build native/build/ruckig -j2
cmake --install native/build/ruckig
cmake -S vendor/franky -B native/build/franky -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PYTHON_STUBS=OFF \
    -DCMAKE_PREFIX_PATH="$project/.franka-player/native;$project/.franka-player/build-tools/pybind11" \
    -DPYTHON_EXECUTABLE="$project/.venv/bin/python" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON -DCMAKE_INSTALL_RPATH='$ORIGIN'
cmake --build native/build/franky -j2
.venv/bin/python - <<'PY'
import hashlib, json, shutil, sysconfig
from pathlib import Path
root=Path.cwd()
destination=root/'.franka-player/native/python/franky'
shutil.copytree(root/'vendor/franky/franky', destination, dirs_exist_ok=True)
for path in (root/'native/build/franky').glob('*.so'):
    staged=destination/(path.name+".new")
    shutil.copy2(path, staged)
    staged.replace(destination/path.name)
shutil.copy2(root/'vendor/franky/LICENSE', destination/'LICENSE')
site=Path(sysconfig.get_paths()['purelib'])
(site/'robol2l-franky.pth').write_text(str(destination.parent)+'\n')
files=['native/franky_station.cpp','native/sampled_joint_curve.h','patches/franky-station.patch']
files += [str(p.relative_to(root)) for p in destination.glob('*.so')]
manifest={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in files}
(destination.parent/'franky-build.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('Installed local franky binding:', destination)
PY
.venv/bin/python -c 'import franky; assert hasattr(franky, "StationJointTrajectory"); print("franky import OK; no hardware connection")'
