"""Read-only device acquisition. Camera SDKs and native helpers are project-local."""
from __future__ import annotations
import importlib
import ctypes
import json
from pathlib import Path
import subprocess
import sys
import time
import cv2
import numpy as np
from .common import ROOT, read_json, write_json, digest
from .geometry import matrix, pose_dict


def native(name, *args, timeout=30):
    binary = ROOT / "native/build" / name
    if not binary.is_file():
        raise RuntimeError(f"Missing native helper {name}; run scripts/build-native.sh")
    p = subprocess.run([str(binary), *map(str, args)], capture_output=True, text=True, timeout=timeout)
    if p.returncode:
        raise RuntimeError(p.stderr.strip() or f"{name} failed: {p.returncode}")
    return p.stdout


def native_receipt(name, *args, timeout=30):
    """Preserve a gripper's explicit negative receipt even on nonzero exit."""
    if name != 'franka_cap_gripper':
        raise ValueError('unsupported_receipt_helper')
    p = subprocess.run([str(ROOT/'native/build'/name), *map(str, args)],
                       capture_output=True, text=True, timeout=timeout)
    try:
        result = json.loads(p.stdout)
        if type(result['ok']) is not bool or type(result['is_grasped']) is not bool or type(result['stopped']) is not bool:
            raise ValueError('invalid_gripper_receipt')
        if not np.isfinite(result['width_m']) or not 0 <= result['width_m'] <= .08:
            raise ValueError('invalid_gripper_width')
        if p.returncode and result['ok']:
            raise ValueError('contradictory_gripper_receipt')
        return result
    except (ValueError, KeyError, TypeError) as error:
        raise RuntimeError('native_gripper_receipt_unavailable') from error


def robot_state(cfg):
    state = json.loads(native("franka_cap_state", cfg["robot_ip"], timeout=10))
    state.update(timestamp_s=time.time(), host_monotonic_s=time.monotonic(), source="physical_FR3", synthetic=False)
    modes = ["other", "idle", "move", "guiding", "reflex", "user_stopped", "automatic_error_recovery"]
    state["mode_name"] = modes[state["mode"]] if 0 <= state["mode"] < len(modes) else "unknown"
    state["end_effector_pose"] = pose_dict(state["T_base_ee"])
    return state


def calibration(cfg, name, state):
    path = cfg[name + "_calibration"]
    cal = read_json(path)
    if cal.get("camera_serial") != cfg[name + "_serial"] or cal.get("units") != "meters":
        raise ValueError(f"{name} calibration serial/units mismatch")
    if name == "wrist":
        if cal.get("parent_frame") != "EE" or cal.get("status") != "validated":
            raise ValueError("Wrist calibration not validated for configured EE")
        transform = matrix(state["T_base_ee"]) @ matrix(cal["T_EE_camera"])
    else:
        if cal.get("parent_frame") != "base" or cal.get("status") != "active_runtime_configuration":
            raise ValueError("Global camera calibration is not active")
        transform = matrix(cal["T_base_camera"])
    return cal, transform, digest(path)


def _check_intrinsics(meta, cal):
    reference = cal["intrinsics"]
    if any(meta[k] != reference[k] for k in ("width", "height")):
        raise ValueError("Camera resolution differs from calibration")


def _realsense():
    # Only the SDK binary staged by bootstrap is used; no imports from older projects.
    bindings = ROOT / "vendor/realsense/python"
    if not bindings.is_dir():
        raise RuntimeError("Project-local RealSense bindings missing")
    ctypes.CDLL(str(ROOT / "vendor/realsense/lib/librealsense2.so.2.58"), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(bindings))
    return importlib.import_module("pyrealsense2")


def wrist_capture(cfg, directory):
    rs = _realsense()
    if rs.__version__ != "2.58.1":
        raise RuntimeError("Expected RSUSB librealsense 2.58.1")
    pipe = rs.pipeline(); spec = rs.config()
    spec.enable_device(cfg["wrist_serial"])
    spec.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    spec.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipe.start(spec)
    try:
        for _ in range(30): pipe.wait_for_frames(3000)
        align = rs.align(rs.stream.color)
        frames = align.process(pipe.wait_for_frames(3000))
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth: raise RuntimeError("Missing RGB-D frame")
        stamp = time.monotonic()
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        k = color.profile.as_video_stream_profile().intrinsics
        meta = {key: getattr(k, key) for key in ("width", "height", "fx", "fy", "ppx", "ppy")}
        meta.update(serial=profile.get_device().get_info(rs.camera_info.serial_number),
                    coeffs=list(k.coeffs), model=str(k.model), frame_id=color.frame_number,
                    depth_frame_id=depth.frame_number, color_timestamp_ms=color.timestamp,
                    depth_timestamp_ms=depth.timestamp, timestamp_domain=str(color.get_frame_timestamp_domain()),
                    host_monotonic_s=stamp, depth_scale_m=scale, depth_available=True,
                    alignment="depth_to_color", sdk_version=rs.__version__)
        rgb = np.asanyarray(color.get_data()).copy()
        depth_m = np.asanyarray(depth.get_data()).astype(np.float32) * scale
        if not cv2.imwrite(str(directory / "rgb.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
            raise RuntimeError("Failed to save wrist RGB")
        np.save(directory / "depth_m.npy", depth_m, allow_pickle=False)
        write_json(directory / "camera.json", meta)
        return meta
    finally:
        pipe.stop()


def global_capture(cfg, directory):
    native("franka_cap_global", cfg["global_serial"], directory, timeout=30)
    meta = read_json(directory / "camera.json")
    if not meta.get("depth_available") or meta.get("alignment") != "depth_to_color":
        raise ValueError("Global camera did not produce aligned depth")
    scale = meta["depth_scale_m"]
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Invalid global depth scale")
    raw = np.fromfile(directory / "depth_u16.raw", dtype=np.uint16)
    if raw.size != meta["height"] * meta["width"]:
        raise ValueError("Global depth file has incorrect size")
    depth_m = raw.reshape(meta["height"], meta["width"]).astype(np.float32) * scale
    if not np.any(depth_m > 0):
        raise ValueError("Global depth frame contains no valid distances")
    np.save(directory / "depth_m.npy", depth_m, allow_pickle=False)
    return meta


def capture(cfg, directory, names=("wrist", "global")):
    if isinstance(names, str): names = [names]
    if not names or len(set(names)) != len(names) or any(x not in ("wrist", "global") for x in names):
        raise ValueError("Camera names must be wrist and/or global, without duplicates")
    directory = Path(directory); directory.mkdir()
    before = robot_state(cfg)
    for name in names: calibration(cfg, name, before)
    cameras = {}
    # Sequential observations are explicit; no claim of hardware synchronization.
    for name in names:
        dest = directory / name; dest.mkdir()
        if name == "wrist":
            meta = wrist_capture(cfg, dest); image = dest / "rgb.png"
        else:
            meta = global_capture(cfg, dest); image = dest / "rgb.jpg"
        if meta["serial"] != cfg[name + "_serial"]: raise ValueError("Camera serial changed")
        cameras[name] = dict(meta, rgb_path=str(image), depth_path=str(dest / "depth_m.npy") if meta["depth_available"] else None)
    after = robot_state(cfg)
    if before["has_errors"] or after["has_errors"]:
        raise RuntimeError("FR3 reports errors; observation not valid for planning")
    for name, cam in cameras.items():
        cal, transform, sha = calibration(cfg, name, after)
        _check_intrinsics(cam, cal)
        cam.update(T_base_camera=transform.tolist(), frame=name + "_color_optical", calibration_sha256=sha,
                   calibration_status=cal["status"], calibration_basis=cal.get("validation_basis", cal.get("note")))
    report = {"id": directory.name, "directory": str(directory), "synthetic": False, "fresh": True,
              "timestamp_s": time.time(), "host_monotonic_s": time.monotonic(), "robot": after,
              "state_bracket_monotonic_s": [before["host_monotonic_s"], after["host_monotonic_s"]],
              "cameras": cameras, "synchronization": "sequential_state_bracket", "source": "physical_devices",
              "measurement_diagnostics": {"joint_delta": (np.array(after['q'])-before['q']).tolist(),
                                          "before_velocities": before['dq'], "after_velocities": after['dq']}}
    write_json(directory / "observation.json", report)
    return report
