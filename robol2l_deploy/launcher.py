"""Local experiment launcher. One fixed-command Runtime process at a time."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from uuid import uuid4
from urllib.parse import urlparse
import requests
from franka_player.server import Server, handler
from franka_player.core import Console
from .cap.storage import write_json

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).with_name('web')
MODES = ('mock', 'panda_py', 'franky', 'arx_x5_2023_dual')


class HarnessLauncher:
    def __init__(self, directory, station, runtime_port=8771, launcher_port=8770):
        self.directory = Path(directory).resolve();self.directory.mkdir(parents=True, exist_ok=True)
        self.station = Path(station).resolve()
        self.runtime_port, self.launcher_port = runtime_port, launcher_port
        self.client = requests.Session();self.client.trust_env = False
        self.lock = threading.RLock();self.process = None
        path = self.directory/'launcher.json'
        self.record = json.loads(path.read_text()) if path.exists() else {}

    def save(self, action):
        write_json(self.directory/'launcher.json', self.record)
        with (self.directory/'interventions.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(action=action, time=time.time(), **self.record))+'\n')

    def probe(self):
        if not self.record.get('active'):return None
        try:
            response = self.client.get(self.record['url']+'/api/hardware/session', timeout=2)
            response.raise_for_status();state = response.json()
            if state.get('session_id') == self.record.get('session_id'):return state
        except requests.RequestException:pass
        return None

    def snapshot(self):
        with self.lock:
            state = self.probe()
            running = bool(state or (self.process and self.process.poll() is None))
            return dict(running=running, session=self.record, runtime=state,
                modes=list(MODES), resumes_automatically=False, lifecycle_protocol='resident_arx_owner_v1')

    def connect(self, backend, *, hardware_supported=False):
        """Select an idle backend in one user action; repeated connects are idempotent."""
        if backend not in MODES:raise ValueError('unknown_backend')
        with self.lock:
            current=self.snapshot()
            switching_held_arx=(self.record.get('backend')=='arx_x5_2023_dual' and backend!='arx_x5_2023_dual'
                                and not self.record.get('hardware_disconnected'))
            if switching_held_arx and not hardware_supported:
                raise ValueError('hardware_support_confirmation_required_before_backend_switch')
            if switching_held_arx:
                self.maintenance()
                current=self.snapshot()
            if current['running']:
                state=current['runtime'] or {}
                if state.get('busy') or state.get('cap_running'):raise ValueError('station_busy')
                if self.record.get('backend')==backend and state and not state.get('closed'):
                    return current
                self.stop(hardware_supported=hardware_supported)
            return self.start(backend)

    def restart(self, *, hardware_supported=False):
        """Reload installed code, retaining receipts and never resuming actions."""
        with self.lock:
            state=self.snapshot()
            if not state['running']:raise ValueError('no_current_session')
            if not state['runtime'] or state['runtime'].get('busy') or state['runtime'].get('cap_running'):
                raise ValueError('station_busy')
            backend,folder=self.record['backend'],self.record['directory']
            lifecycle=state['runtime'].get('hardware_lifecycle') or {}
            if backend=='arx_x5_2023_dual' and lifecycle.get('mode')!='resident_arx_owner_v1' and not hardware_supported:
                raise ValueError('hardware_support_confirmation_required_legacy_runtime')
            self.stop(hardware_supported=hardware_supported)
            result=self.start(backend,history_directory=folder)
            self.save('reload_code_keep_receipts_no_action_resume')
            return result

    def maintenance(self, *, reload=False):
        with self.lock:
            current=self.snapshot()
            if current['runtime'] and (current['runtime'].get('busy') or current['runtime'].get('cap_running')):
                raise ValueError('station_busy')
            if reload and current['running']:
                return self.restart(hardware_supported=True)
            self.stop(hardware_supported=True)
            from .arx_owner import stop_unattached_owner
            stop_unattached_owner()
            self.record['hardware_disconnected']=True
            self.save('supported_hardware_maintenance_completed')
            if reload:
                return self.start(self.record['backend'],history_directory=self.record['directory'])
            return self.snapshot()

    def start(self, backend, *, history_directory=None):
        if backend not in MODES:raise ValueError('unknown_backend')
        with self.lock:
            if self.snapshot()['running']:raise ValueError('end_the_current_experiment_first')
            # Never attach to or terminate an unrelated listener at this port.
            with socket.socket() as check:
                # The previous HTTP session may leave TIME_WAIT sockets. Match
                # HTTPServer's reuse policy, while still rejecting a live listener.
                check.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                check.bind(('127.0.0.1', self.runtime_port))
            key = uuid4().hex
            folder=Path(history_directory) if history_directory else self.directory/'sessions'/key
            folder.mkdir(parents=True,exist_ok=history_directory is not None)
            env = dict(os.environ, ROBOL2L_LAUNCHER_URL=f'http://127.0.0.1:{self.launcher_port}',
                ROBOL2L_MANAGED_SESSION=key)
            command = [sys.executable, '-m', 'franka_player.server', '--port', str(self.runtime_port),
                '--data-dir', str(folder)]
            if backend == 'mock':command += ['--hardware-mock']
            elif backend == 'arx_x5_2023_dual':
                sdk = env.get('ARX5_SDK_DIR') or str(ROOT.parent / 'ARX5_beta')
                env['PYTHONPATH'] = os.pathsep.join(filter(None, (sdk, env.get('PYTHONPATH'))))
                command += ['--hardware-station', str(ROOT / 'config/arx-x5-2023-dual.json'),
                            '--robot-backend', backend, '--hardware-connect']
            else:command += ['--hardware-station', str(self.station), '--robot-backend', backend]
            with (folder/'server.log').open('ab') as log:
                self.process = subprocess.Popen(command, cwd=ROOT, env=env,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.record = dict(active=True, backend=backend, session_id=key, pid=self.process.pid,
                url=f'http://127.0.0.1:{self.runtime_port}', directory=str(folder))
            self.save('start_read_only_session')
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                if self.probe():
                    if backend != 'mock':
                        from .providers.services import ensure_local_services
                        ensure_local_services(include_grasps=backend != 'arx_x5_2023_dual')
                        # Selecting a physical backend explicitly connects it.
                        # Use the existing read-only API and retain its feedback.
                        response = self.client.post(self.record['url']+'/api/hardware/operator/call',
                            json=dict(request_id='startup-connect-'+key, operation='get_observation', arguments={}),
                            headers={'X-Console-Request':'1','X-Session-Id':key}, timeout=5)
                        response.raise_for_status()
                    return self.snapshot()
                if self.process.poll() is not None:break
                time.sleep(.1)
            raise RuntimeError('runtime_start_failed; inspect '+str(folder/'server.log'))

    def stop(self, *, hardware_supported=False):
        with self.lock:
            state = self.probe()
            if state is None:
                if self.process and self.process.poll() is None:
                    raise ValueError('runtime_unresponsive; no automatic termination')
                self.record['active'] = False;self.save('session_already_stopped');return self.snapshot()
            if not state.get('closed'):
                lifecycle=state.get('hardware_lifecycle') or {}
                if self.record.get('backend')=='arx_x5_2023_dual' and lifecycle.get('mode')!='resident_arx_owner_v1' and not hardware_supported:
                    raise ValueError('hardware_support_confirmation_required_legacy_runtime')
                body={'confirmed':True}
                if hardware_supported:
                    self.save('operator_confirmed_both_arms_supported_for_hardware_shutdown')
                    # Old Runtime accepts only the original body. Never send it without the new explicit confirmation.
                    if lifecycle:body['hardware_supported']=True
                response = self.client.post(self.record['url']+'/api/hardware/session/end',
                    json=body, headers={'X-Console-Request':'1','X-Session-Id':self.record['session_id']}, timeout=10)
                if not response.ok:raise ValueError(response.json().get('error', 'runtime_busy'))
            deadline = time.monotonic()+25
            while time.monotonic() < deadline:
                if self.process:
                    if self.process.poll() is not None:break
                elif not self.probe():break
                time.sleep(.1)
            else:raise ValueError('runtime_still_closing; do_not_start_another_controller')
            self.record['active'] = False;self.save('end_session')
            return self.snapshot()


def launcher_handler(base, manager):
    class Handler(base):
        def do_GET(self):
            if not self.trusted():return self.send(403, {'error':'origin_rejected'})
            route = urlparse(self.path).path
            if route == '/api/launcher/state':return self.send(200, manager.snapshot())
            if route == '/panel':
                # One bookmarkable address: straight into a live session, or to the
                # connect page when none is running. Never connects on its own.
                state = manager.snapshot()
                url = (state.get('session') or {}).get('url') if state.get('running') else None
                self.send_response(302)
                self.send_header('Location', (url + '/hardware') if url else '/')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                return
            assets = {'/':('launcher.html','text/html'), '/launcher.js':('launcher.js','text/javascript'),
                '/hardware.css':('style.css','text/css')}
            if route not in assets:return self.send(404, {'error':'not_found'})
            name, kind = assets[route];return self.send(200, (ASSETS/name).read_bytes(), kind)

        def do_POST(self):
            if not self.trusted() or self.headers.get('X-Console-Request') != '1':
                return self.send(403, {'error':'origin_rejected'})
            try:
                length = int(self.headers.get('Content-Length','0'))
                if length <= 0:raise ValueError('invalid_body')
                body = json.loads(self.rfile.read(length));route = urlparse(self.path).path
                if route == '/api/launcher/start' and set(body) == {'backend'}:
                    return self.send(200, manager.start(body['backend']))
                if route == '/api/launcher/connect' and set(body) == {'backend'}:
                    return self.send(200, manager.connect(body['backend']))
                if route == '/api/launcher/reload' and body == {}:
                    return self.send(200, manager.restart())
                if route == '/api/launcher/maintenance/reload' and body == {'hardware_supported':True}:
                    return self.send(200, manager.maintenance(reload=True))
                if route == '/api/launcher/maintenance/stop' and body == {'hardware_supported':True}:
                    return self.send(200, manager.maintenance())
                if route == '/api/launcher/stop' and body == {'confirmed':True}:
                    return self.send(200, manager.stop())
                raise ValueError('invalid_request')
            except Exception as error:return self.send(409, {'error':str(error)})
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8770)
    parser.add_argument('--runtime-port', type=int, default=8771)
    parser.add_argument('--station', default=str(ROOT/'config/station.json'))
    parser.add_argument('--data-dir', default=str(ROOT/'.franka-player/panel'))
    args = parser.parse_args()
    console = Console(Path(args.data_dir)/'launcher-console',model_settings_path=Path(args.data_dir)/'model-settings.json')
    manager = HarnessLauncher(args.data_dir, args.station, args.runtime_port, args.port)
    server = Server(('127.0.0.1', args.port), launcher_handler(handler(console), manager))
    print(f'RoboL2L panel: http://127.0.0.1:{args.port} · no hardware selected', flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close();console.shutdown()


if __name__ == '__main__':main()
