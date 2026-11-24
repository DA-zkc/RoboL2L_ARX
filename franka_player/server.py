"""Loopback-only console; use SSH forwarding for remote access."""
import argparse
import importlib
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from .core import Console
from .bundle import MAX_ARCHIVE

WEB = Path(__file__).with_name("web")
class Server(ThreadingHTTPServer):
    daemon_threads = True

def handler(console):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass
        def send(self, code, value, content_type="application/json"):
            data = json.dumps(value, ensure_ascii=False).encode() if content_type == "application/json" else value
            self.send_response(code)
            self.send_header("Content-Type", content_type + "; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            self.wfile.write(data)
        def trusted(self):
            host = self.headers.get("Host", "")
            if host.split(":")[0] not in ("127.0.0.1", "localhost"):
                return False
            origin = self.headers.get("Origin")
            return not origin or origin == "http://" + host
        def do_GET(self):
            if not self.trusted():
                return self.send(403, {"error": "origin_rejected"})
            path = urlparse(self.path).path
            if path == "/api/state":
                return self.send(200, console.snapshot())
            if path == "/api/export":
                return self.send(200, console.snapshot())
            assets = {"/": ("index.html", "text/html"), "/app.js": ("app.js", "text/javascript"),
                      "/style.css": ("style.css", "text/css"), "/cap.js": ("cap.js", "text/javascript")}
            if path not in assets:
                return self.send(404, {"error": "not_found"})
            name, kind = assets[path]
            self.send(200, (WEB / name).read_bytes(), kind)
        def do_POST(self):
            if not self.trusted() or self.headers.get("X-Console-Request") != "1":
                return self.send(403, {"error": "origin_rejected"})
            try:
                route = urlparse(self.path).path
                length = int(self.headers.get("Content-Length", "0"))
                if route == "/api/experiments/import":
                    if not 0 < length <= MAX_ARCHIVE:
                        raise ValueError("bundle_size_limit")
                    console.import_bundle(self.rfile.read(length))
                    return self.send(200, console.snapshot())
                if not 0 < length <= 512000:
                    raise ValueError("invalid_body_size")
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict):
                    raise ValueError("invalid_body")
                route = urlparse(self.path).path
                if route == "/api/experiments":
                    console.create(body)
                elif route == "/api/select":
                    console.select(body.get("id"), body.get("next", False))
                elif route == "/api/config":
                    console.configure(body)
                elif route == "/api/start":
                    if body.get("confirmed") is not True:
                        raise ValueError("confirmation_required")
                    console.start()
                elif route == "/api/resume":
                    console.start(resume=True, operator_message=body.get("message", ""))
                elif route == "/api/pause":
                    console.pause()
                elif route == "/api/stop":
                    console.stop()
                else:
                    return self.send(404, {"error": "not_found"})
                self.send(200, console.snapshot())
            except (ValueError, KeyError, TypeError, AttributeError) as exc:
                error = str(exc) if isinstance(exc, ValueError) and str(exc).replace("_","").isalnum() else "invalid_request"
                self.send(409, {"error": error})
            except Exception:
                self.send(500, {"error": "internal_error"})
    return Handler

def main():
    from robol2l_deploy.adapters import STATION_BACKENDS, make_station_adapter

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--data-dir", default=".franka-player")
    parser.add_argument("--runtime-factory", help="Optional installed module:function returning a real Runtime")
    parser.add_argument("--cap-station", help="Project-owned CAP station configuration (observation/planning only)")
    parser.add_argument("--motion-profile", help="Explicit local authorization and guarded scene for a single physical task")
    parser.add_argument('--hardware-station', help='Opt-in station for the modular hardware workbench')
    parser.add_argument('--robot-backend', choices=STATION_BACKENDS, default='panda_py', help='Select one library before the physical experiment')
    parser.add_argument('--hardware-mock', action='store_true', help='Synthetic hardware workbench; no device access')
    parser.add_argument('--hardware-connect', action='store_true', help='Permit opening the installed SDK for sensing; ARX initialization can energize both arms')
    parser.add_argument('--hardware-motion', action='store_true', help='Explicitly enable the installed hardware Runtime motion gate')
    args = parser.parse_args()
    if (args.hardware_station or args.hardware_mock) and (args.cap_station or args.runtime_factory or args.motion_profile):
        parser.error('Choose one installed station Runtime; legacy and hardware modes cannot coexist')
    if args.hardware_station and args.hardware_mock: parser.error('Choose physical or mock hardware')
    if args.hardware_motion and not (args.hardware_station or args.hardware_mock): parser.error('--hardware-motion requires a hardware station')
    if args.motion_profile and not args.cap_station:parser.error("--motion-profile requires --cap-station")
    factory = None
    if args.runtime_factory:
        module, name = args.runtime_factory.split(":", 1)
        factory = getattr(importlib.import_module(module), name)
    if args.cap_station:
        from franka_cap.sdk import CapSDK
        from franka_cap.runtime import FrankaRuntime
        factory = lambda config: FrankaRuntime(CapSDK(args.cap_station, motion_profile=args.motion_profile))
    model_settings=None
    if os.environ.get('ROBOL2L_MANAGED_SESSION'):
        model_settings=Path(args.data_dir).resolve().parent.parent/'model-settings.json'
    console = Console(args.data_dir, factory,model_settings_path=model_settings)
    sdk = None
    selected_handler = handler(console)
    if args.cap_station:
        from franka_cap.sdk import CapSDK
        from franka_cap.web import cap_handler
        sdk = CapSDK(args.cap_station)
        selected_handler = cap_handler(console, sdk)
    hardware = None
    if args.hardware_station or args.hardware_mock:
        from robol2l_deploy.runtime import StationRuntime
        from robol2l_deploy.web import hardware_handler
        if args.hardware_mock:
            from robol2l_deploy.adapters.mock import MockAdapter
            adapter = MockAdapter()
        else:
            if args.robot_backend == 'arx_x5_2023_dual' and (args.hardware_connect or args.hardware_motion):
                from robol2l_deploy.arx_owner import ensure_owner
                from robol2l_deploy.adapters.arx_dual import DualARXStation
                owner_socket = ensure_owner(args.hardware_station, Path(args.hardware_station).resolve().parents[1]/'.franka-player/arx-owner')
                adapter = DualARXStation.from_file(args.hardware_station, allow_connection=True,
                    allow_motion=False, owner_socket=owner_socket)
                if args.hardware_motion:
                    adapter.allow_motion = True
            else:
                adapter = make_station_adapter(args.hardware_station, args.robot_backend,
                    allow_connection=args.hardware_connect or args.hardware_motion, allow_motion=args.hardware_motion)
        hardware = StationRuntime(adapter, Path(args.data_dir)/'hardware', allow_motion=args.hardware_motion)
        selected_handler = hardware_handler(selected_handler, hardware, model_config=lambda: console.snapshot()["config"],
            experiment_root=model_settings.parent if model_settings else Path(args.data_dir))
    server = Server(("127.0.0.1", args.port), selected_handler)
    print(f"Franka Player: http://127.0.0.1:{args.port} | hardware={'configured (motion disabled)' if args.hardware_station and not args.hardware_motion else 'mock' if args.hardware_mock else 'configured' if factory or hardware else 'not connected'}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        console.shutdown()
        if hardware is not None:
            hardware.close()
        if sdk is not None:
            sdk.close()

if __name__ == "__main__":
    main()
