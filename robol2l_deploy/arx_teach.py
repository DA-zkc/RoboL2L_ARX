"""One manual positioning session, one arm, one exclusive runtime lease."""
import threading
import time
import numpy as np
from uuid import uuid4

from .physical import authorize, revoke
from .runtime import teaching_fault


class TeachKey:
    delay_s = 10.

    def __init__(self, runtime):
        self.runtime = runtime
        self.lock = threading.Condition()
        self.token = None
        self.released = False
        self.phase = 'idle'
        self.side = None
        self.error = None
        self.error_details = {}
        self.receipt = None
        self.mode = 'key'
        self.start_at = None
        self.exit_reason = None
        if runtime.faulted and runtime.teach_fault:
            self.phase = 'error'
            self.side = runtime.teach_fault['side']
            self.error = runtime.teach_fault['error']
            self.receipt = runtime.teach_fault.get('receipt')

    def acknowledge_parked(self, side, *, recheck_hold=False):
        rt = self.runtime
        with rt.lease(), self.lock:
            kind = (rt.teach_fault or {}).get('kind')
            if (self.token or rt.paused or not rt.faulted or not rt.teach_fault
                    or rt.teach_fault['side'] != side or kind not in (('feedback',) if recheck_hold else ('limit','verified'))):
                raise ValueError('no_matching_teach_fault')
            # Keep the runtime reserved throughout the stationary feedback window.
            first = rt.get_robot_state()
            receipt = rt.teach_fault.get('receipt')
            def validate(state):
                rt.adapter.check_teach_parked(first, state)
            validate(first)
            if recheck_hold:
                validate(rt.get_robot_state())
                driver = rt.adapter.arms[side].driver
                if rt.allow_motion or driver.allow_motion or not rt.adapter.realtime_ready():
                    raise ValueError('parked_recheck_requires_connected_motion_disabled_station')
                # Consume the one attempt durably. The Runtime stays faulted until
                # the selected arm has a verified receipt and both arms pass readback.
                rt.event('teach_recheck_dispatch', arm=side, reason='operator_confirmed_both_arms_supported')
                rt.teach_fault = None
                try:
                    # The resident owner has its own permission; changing only
                    # the selected proxy driver's flag cannot authorize its RPC.
                    rt.adapter.allow_motion = True
                    receipt = rt.adapter.teach_hold(side, parked_reference=first.joint_positions[side])
                    rt.event('teach_recheck_held', arm=side, receipt=receipt)
                    self.error = 'native_hold_command_completed'
                    self.receipt = receipt
                    rt.teach_fault = dict(kind='verified',side=side,receipt=receipt,error=self.error)
                except Exception as error:
                    self.error = str(error)
                    self.error_details = dict(getattr(error, 'details', {}))
                    rt.event('outcome_unknown', operation='teach_recheck', arm=side,
                             error=self.error, details=self.error_details)
                    raise
                finally:
                    rt.adapter.allow_motion = False
                # Confirm stationarity around the settled pose. The hold may
                # have taken up compliance between the preflight and its receipt.
                first = rt.get_robot_state()
                validate(first)
            rt.acknowledge_fault('operator_confirmed_both_arms_supported_after_' + side + '_teach_' + kind,
                                 validate=validate)
            self.phase, self.side, self.error, self.receipt = (
                ('held', side, None, receipt) if recheck_hold else ('idle', None, None, None))
            return self.status()

    def status(self):
        with self.lock:
            fault = self.runtime.teach_fault if self.runtime.faulted else None
            return dict(phase=self.phase, side=self.side, error=self.error, receipt=self.receipt,
                        active=bool(self.token), mode=self.mode, exit_reason=self.exit_reason,
                        countdown_s=max(0., self.start_at-time.monotonic()) if self.phase=='countdown' and self.start_at else None,
                        error_details=dict(self.error_details),
                        can_acknowledge=bool(fault and fault['kind'] in ('limit','verified')),
                        can_recheck_hold=bool(fault and fault['kind']=='feedback'))

    def prepare(self, side, *, delayed=False):
        if side not in ('left', 'right'):
            raise ValueError('invalid_arm')
        with self.lock:
            if self.token:
                raise ValueError('teach_key_busy')
            if delayed and (self.runtime.closed or self.runtime.faulted or self.runtime.paused):
                raise ValueError('station_faulted_or_paused')
            owner = getattr(self.runtime.adapter, 'owner_client', None)
            if delayed and owner and owner.call('heartbeat').get('manual_positioning_version', 0) < 1:
                raise ValueError('manual_positioning_requires_owner_maintenance_update')
            self.token = token = uuid4().hex
            self.side, self.phase, self.error, self.receipt = side, 'armed', None, None
            self.mode = 'delayed' if delayed else 'key'
            self.start_at = time.monotonic() + self.delay_s if delayed else None
            self.exit_reason = None
            self.error_details = {}
            self.released = False
            threading.Thread(target=self._run, args=(side,), daemon=True).start()
            return dict(token=token)

    def hold(self):
        """Explicit panel stop, also usable after navigation/reopening the page."""
        with self.lock:
            if self.token:
                self.released = True
                self.lock.notify_all()
            return self.status()

    def signal(self, token, release):
        with self.lock:
            if not self.token or self.token != token:
                raise ValueError('teach_key_expired')
            self.released |= release
            if not self.released and self.phase == 'armed':
                self.phase = 'pressed'
            self.lock.notify_all()

    def _run(self, side):
        rt = self.runtime
        try:
            with rt.program_lease():
                with self.lock:
                    if self.mode == 'delayed':
                        self.phase = 'countdown'
                        rt.event('teach_countdown', arm=side, delay_s=self.delay_s)
                        while time.monotonic() < self.start_at:
                            if self.released or rt.paused or rt.closed or rt.faulted:
                                self.phase = 'cancelled'
                                rt.event('teach_cancelled', arm=side)
                                return
                            self.lock.wait(min(.08, max(0., self.start_at-time.monotonic())))
                    else:
                        self.lock.wait_for(lambda: self.phase == 'pressed' or self.released)
                        if self.phase != 'pressed' or self.released:
                            self.phase = 'cancelled'
                            return
                authorize(rt, 'arx-manual-teach', 'Operator requested '+self.mode+' manual positioning for '+side)
                try:
                    rt.adapter.check_teach_start(side)
                    with self.lock:
                        if (self.released or False
                                or rt.paused or rt.closed or rt.faulted):
                            self.phase = 'cancelled'
                            return
                    rt.event('teach_dispatch', arm=side, mode=self.mode)
                    try:
                        rt.adapter.teach_drag(side)
                        with self.lock:
                            self.phase = 'drag'
                            while not (self.released or rt.paused or rt.closed or rt.faulted):
                                remaining = .08
                                if remaining <= 0:
                                    break
                                self.lock.wait(min(.08, remaining))
                            self.exit_reason = ('operator_hold' if self.released else 'runtime_closed' if rt.closed
                                                else 'runtime_fault' if rt.faulted else 'runtime_pause' if rt.paused
                                                else 'heartbeat_expired')
                    finally:
                        with self.lock:
                            self.phase = 'locking'
                        rt.event('teach_hold_requested', arm=side, reason=self.exit_reason)
                        receipt = rt.adapter.teach_hold(side)
                        rt.event('teach_held', arm=side, receipt=receipt)
                        with self.lock:
                            self.receipt, self.phase = receipt, 'held'
                finally:
                    revoke(rt)
        except Exception as error:
            with self.lock:
                self.error_details = dict(getattr(error,'details',{}))
                if self.phase == 'locking':
                    rt.teach_fault = None if rt.faulted else teaching_fault(side, str(error))
                    rt.faulted = True
                    rt.event('outcome_unknown', operation='teach_hold', arm=side,
                             error=str(error), details=self.error_details)
                if self.phase in ('pressed', 'countdown'):
                    rt.event('teach_start_failed', arm=side, error=str(error), details=self.error_details)
                self.phase, self.error = 'error', str(error)
        finally:
            with self.lock:
                self.token = None
                self.lock.notify_all()
