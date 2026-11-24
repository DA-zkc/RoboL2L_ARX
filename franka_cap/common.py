from __future__ import annotations

import contextlib
import fcntl
import hashlib
import json
import math
from pathlib import Path
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + ".tmp")
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    temp.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scalar(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}: finite number required")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f"{name}: outside [{low}, {high}]")
    return float(value)


def config(path=None):
    path = Path(path or ROOT / "config/station.json").resolve()
    cfg = read_json(path)
    if cfg.get("schema") != 1 or cfg.get("station") != "fr3-single":
        raise ValueError("Unsupported station configuration")
    # Runtime owns motion activation; station files keep startup read-only.
    if cfg.get("motion_enabled") is not False:
        raise ValueError("Motion activation belongs to Runtime; station motion_enabled must be false")
    cfg["config_path"] = str(path)
    for key in ("wrist_calibration", "global_calibration"):
        cfg[key] = str((path.parent / cfg[key]).resolve())
    return cfg


def new_run(label="session"):
    path = ROOT / "runs/cap" / (time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + label + "-" + uuid4().hex[:8])
    path.mkdir(parents=True)
    return path


@contextlib.contextmanager
def hardware_lock():
    path = ROOT / ".franka-player/hardware.lock"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("Station busy: another SDK request owns the devices") from None
        try:
            yield
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
