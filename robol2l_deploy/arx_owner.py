"""Resident hardware portion of the ARX Runtime. Fixed SDK RPC; no generated code.

The console owns planning/experiments; this process alone owns both SDK loops.
Console loss revokes dispatch and requests healthy JOINT controllers to hold.
Only explicit supported-hardware maintenance may close the SDK handles.
"""
import argparse
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import signal
import select
import socket
import socketserver
import struct
import subprocess
import sys
import threading
import time
from uuid import uuid4

import numpy as np

from .runtime import public

READS = {'get_status', 'get_joint_positions', 'get_joint_velocities', 'get_gripper_pos', 'get_gripper_vel'}
WRITES = {'set_joint_positions', 'set_gripper_pos', 'gravity_compensation'}
PROPERTIES = {'fault', 'offline_joints'}


class OwnerRejected(RuntimeError):
    pass


def socket_path():
    folder = Path('/tmp') / f'robol2l-arx-{os.getuid()}'
    folder.mkdir(mode=0o700, exist_ok=True)
    if folder.is_symlink() or folder.stat().st_uid != os.getuid() or folder.stat().st_mode & 0o077:
        raise RuntimeError('insecure_arx_owner_directory')
    return folder / 'owner.sock'


def rpc(path, request, timeout=None):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(str(path))
        sock.sendall(json.dumps(public(request), allow_nan=False).encode() + b'\n')
        with sock.makefile('rb') as stream:
            line = stream.readline()
        if not line:
            raise RuntimeError('arx_owner_response_unknown')
        result = json.loads(line)
        if not result.get('ok'):
            raise OwnerRejected(result.get('error', 'arx_owner_request_failed'))
        return result['result']


class OwnerState:
    """Serialized lease and dispatch state, injectable clock/station for offline tests."""
    def __init__(self, station, *, clock=time.monotonic, journal=lambda event: None):
        self.station, self.clock, self.journal = station, clock, journal
        self.lock = threading.RLock()
        self.identity = station.device_binding()
        self.instance = uuid4().hex
        self.token = None
        self.client_process = None
        self.authorized = False
        self.stopped = False
        self.closing = False
        self.last_hold = {}
        self.teaching = set()

    def status(self):
        return dict(instance=self.instance, identity=self.identity, attached=self.token is not None,
                    authorized=self.authorized, stopped=self.stopped, closing=self.closing, last_hold=self.last_hold,
                    pid=os.getpid(), lifecycle='resident_arx_owner_v1', manual_positioning_version=1,
                    native_query_version=1, client_liveness='kernel_process_exit')

    def lose_client(self, reason):
        from .adapters.arx_support import preserve_support
        self.authorized = False
        self.token = None
        if self.client_process is not None:
            os.close(self.client_process)
            self.client_process = None
        self.last_hold = {}
        for side, unit in self.station.arms.items():
            # Do not create a device handle merely to detach an unused client.
            if unit.driver._arm is not None:
                if side in self.teaching:
                    try:
                        unit.driver.allow_motion = True
                        self.last_hold[side] = dict(strategy='exit_manual_positioning', hold_verified=False,
                                                   receipt=self.station.teach_hold(side))
                    except Exception as error:
                        self.last_hold[side] = dict(strategy='exit_manual_positioning', hold_verified=False,
                                                   reason=str(error), details=getattr(error, 'details', {}))
                    finally:
                        unit.driver.allow_motion = False
                else:
                    self.last_hold[side] = preserve_support(unit, cancel_motion=True)
        self.teaching.clear()
        self.journal(dict(event=reason, hold=public(self.last_hold), instance=self.instance))

    def expire(self):
        with self.lock:
            if self.token and self.client_process is not None and select.select([self.client_process], [], [], 0)[0]:
                self.lose_client('client_process_exited')

    def shutdown(self, event):
        self.authorized = False
        self.closing = True
        self.token = None
        if self.client_process is not None:
            os.close(self.client_process)
            self.client_process = None
        self.journal(dict(event=event, instance=self.instance))
        self.station.close(hardware_supported=True)
        self.stopped = True
        return self.status()

    def handle(self, body):
        with self.lock:
            self.expire()
            op = body.get('op')
            if op == 'status':
                return self.status()
            if self.stopped:
                raise ValueError('arx_owner_stopped')
            if self.closing:
                raise ValueError('arx_owner_shutdown_unconfirmed_no_new_controller')
            if op == 'maintenance_shutdown':
                if (self.token or body.get('instance') != self.instance
                        or body.get('hardware_supported') is not True):
                    raise ValueError('idle_owner_and_support_confirmation_required')
                return self.shutdown('operator_confirmed_supported_unattached_shutdown')
            if op == 'attach':
                if body.get('identity') != self.identity:
                    raise ValueError('arx_owner_model_changed_requires_supported_maintenance')
                if self.token:
                    raise ValueError('arx_owner_already_attached')
                if body.get('_peer_pid') is not None:
                    self.client_process = os.pidfd_open(body['_peer_pid'])
                self.token = uuid4().hex
                self.authorized = False
                self.journal(dict(event='client_attached', instance=self.instance))
                return dict(token=self.token, **self.status())
            if not self.token or body.get('token') != self.token:
                raise ValueError('arx_owner_lease_lost_new_session_required')
            if op == 'heartbeat':
                return self.status()
            if op == 'authorize':
                if type(body.get('enabled')) is not bool:
                    raise ValueError('invalid_motion_permission')
                self.authorized = body['enabled']
                return self.status()
            if op == 'detach':
                self.lose_client('client_detached')
                return self.status()
            if op == 'shutdown':
                if body.get('hardware_supported') is not True:
                    raise ValueError('hardware_support_confirmation_required')
                return self.shutdown('operator_confirmed_supported_hardware_shutdown')
            if op != 'sdk' or body.get('side') not in self.station.arms:
                raise ValueError('invalid_arx_owner_operation')
            name = body.get('method')
            if name == 'native_query':
                from .adapters.arx_native import query
                return query(self.station.arms[body['side']].driver.arm, **body.get('kwargs', {}))
            if name not in READS | WRITES | PROPERTIES:
                raise ValueError('arx_owner_method_not_allowed')
            if name in WRITES and not self.authorized:
                raise ValueError('arx_owner_motion_disabled')
            arm = self.station.arms[body['side']].driver.arm
            if name in PROPERTIES:
                return getattr(arm, name)
            args, kwargs = body.get('args', []), body.get('kwargs', {})
            if name == 'set_joint_positions' and 'positions' in kwargs:
                kwargs['positions'] = np.asarray(kwargs['positions'], dtype=float)
            # Mark before dispatch, so even an exception/lost reply cannot leave
            # an accepted gravity transition invisible to owner lease expiry.
            if name == 'gravity_compensation':
                self.teaching.add(body['side'])
            result = getattr(arm, name)(*args, **kwargs)
            if name == 'set_joint_positions' and result:
                self.teaching.discard(body['side'])
            return result


class OwnerClient:
    def __init__(self, path, identity):
        self.path, self.identity = str(path), identity
        self.token = None
        self.instance = None
        self.manual_positioning_version = 0
        self.lost = False
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.thread = None

    def attach(self):
        with self.lock:
            if self.lost or self.done.is_set():
                raise RuntimeError('arx_owner_connection_lost_new_session_required')
            if self.token is None:
                result = rpc(self.path, dict(op='attach', identity=self.identity))
                self.token, self.instance = result['token'], result['instance']
                self.manual_positioning_version = result.get('manual_positioning_version', 0)
                self.thread = threading.Thread(target=self._heartbeat, daemon=True)
                self.thread.start()

    def call(self, op, **args):
        self.attach()
        try:
            return rpc(self.path, dict(op=op, token=self.token, **args))
        except OwnerRejected:
            raise  # A received rejection is known; do not retry it either.
        except (OSError, TimeoutError, ValueError, RuntimeError):
            self.lost = True
            self.done.set()
            # Never retry an operation whose reply may have been lost.
            raise RuntimeError('arx_owner_reply_unknown_no_retry') from None

    def _heartbeat(self):
        while not self.done.wait(.5):
            try:
                self.call('heartbeat')
            except Exception:
                self.lost = True
                self.done.set()

    def arm(self, side):
        self.attach()
        return RemoteArm(self, side)

    def close(self, *, hardware_supported=False):
        if self.token and not self.lost:
            try:
                self.call('shutdown' if hardware_supported else 'detach',
                          **({'hardware_supported': True} if hardware_supported else {}))
            except Exception:
                # Do not report a maintenance disconnect as completed on a lost reply.
                self.done.set()
                self.lost = True
                raise
            self.done.set()
            self.token = None
        else:
            self.done.set()
            if hardware_supported:
                # Changed-model clients cannot attach. Maintenance can still stop
                # an unattached owner after explicit physical support confirmation.
                status = rpc(self.path, dict(op='status'))
                if not status['stopped']:
                    rpc(self.path, dict(op='maintenance_shutdown', hardware_supported=True,
                                        instance=status['instance']))


class RemoteArm:
    def __init__(self, client, side):
        self.client, self.side = client, side

    def native_query(self, name, **arguments):
        return self.client.call('sdk', side=self.side, method='native_query',
                                kwargs=dict(name=name, **arguments))

    def __getattr__(self, name):
        if name in PROPERTIES:
            return self.client.call('sdk', side=self.side, method=name)
        if name in READS | WRITES:
            def invoke(*args, **kwargs):
                value = self.client.call('sdk', side=self.side, method=name, args=args, kwargs=kwargs)
                return np.asarray(value) if name in {'get_joint_positions', 'get_joint_velocities'} else value
            return invoke
        raise AttributeError(name)


@contextmanager
def owner_lock(path):
    with Path(str(path) + '.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def ensure_owner(station_path, directory):
    """Start a singleton fixed-command service; never replace an existing owner."""
    path = socket_path()
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    # Serialize simultaneous launcher/server starts, separately from the lifetime lock.
    with Path(str(path) + '.startup').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        while True:
            try:
                status = rpc(path, dict(op='status'))
                if not status['stopped']:
                    return str(path)
                time.sleep(.05)
            except OSError:
                break
        try:
            with owner_lock(path):
                pass
        except BlockingIOError:
            raise RuntimeError('arx_owner_unresponsive_do_not_start_another_controller') from None
        env = dict(os.environ)
        root = Path(__file__).resolve().parents[1]
        env['PYTHONPATH'] = os.pathsep.join(filter(None, (str(root.parent/'ARX5_beta'), env.get('PYTHONPATH'))))
        with (directory/'owner.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, '-m', 'robol2l_deploy.arx_owner',
                '--station', str(Path(station_path).resolve()), '--socket', str(path),
                '--directory', str(directory.resolve())], cwd=root, env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while True:
            if process.poll() is not None:
                raise RuntimeError('arx_owner_start_failed_inspect_owner_log')
            try:
                rpc(path, dict(op='status'))
                return str(path)
            except OSError:
                time.sleep(.05)


def serve(state, path):
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(None)
            try:
                line = self.rfile.readline()
                body = json.loads(line)
                # Kernel credentials, never a PID supplied by a remote caller.
                body['_peer_pid'] = struct.unpack('3i', self.connection.getsockopt(
                    socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))[0]
                result = dict(ok=True, result=public(state.handle(body)))
            except Exception as error:
                result = dict(ok=False, error=str(error))
            try:
                self.wfile.write(json.dumps(result, allow_nan=False).encode() + b'\n')
            except OSError:
                pass  # Lost replies are never retried; lease expiry retains support.
    class Server(socketserver.ThreadingUnixStreamServer):
        daemon_threads = True
    with Server(str(path), Handler) as server:
        os.chmod(path, 0o600)
        server.timeout = .1
        while not state.stopped:
            server.handle_request()
            state.expire()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--station', required=True)
    parser.add_argument('--socket', required=True)
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    directory = Path(args.directory)
    def journal(event):
        with (directory/'owner-events.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(time=time.time(), **event)) + '\n')
    with owner_lock(args.socket):
        # A signal must not silently destroy support. Maintenance RPC is the orderly exit.
        def signal_received(signum, frame):
            journal(dict(event='signal_ignored_requires_supported_maintenance', signal=signum))
        signal.signal(signal.SIGINT, signal_received)
        signal.signal(signal.SIGTERM, signal_received)
        from .adapters.arx_dual import DualARXStation
        station = DualARXStation.from_file(args.station, allow_connection=True)
        state = OwnerState(station, journal=journal)
        Path(args.socket).unlink(missing_ok=True)
        while not state.stopped:
            try:
                serve(state, args.socket)
            except Exception as error:
                # Keep SDK threads alive even if the local RPC listener fails.
                with state.lock:
                    try:
                        state.lose_client('owner_listener_failed')
                        journal(dict(event='listener_error', error=str(error)))
                    except Exception:
                        pass
                time.sleep(.5)
                Path(args.socket).unlink(missing_ok=True)
        Path(args.socket).unlink(missing_ok=True)


def stop_unattached_owner():
    """Only called after an explicit supported-hardware maintenance confirmation."""
    path = socket_path()
    try:
        status = rpc(path, dict(op='status'))
    except (FileNotFoundError, ConnectionRefusedError):
        return
    if not status['stopped']:
        rpc(path, dict(op='maintenance_shutdown', hardware_supported=True, instance=status['instance']))


if __name__ == '__main__':
    main()
