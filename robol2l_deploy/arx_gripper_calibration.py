"""Operator-only commissioning through the dual station's existing device owner."""
import json
import math
import time

from .adapters.arx import GRIPPER_RANGE, EMPTY_CLOSED_MAX_RAD
from .cap.storage import write_json
from .physical import authorize, revoke

CLOSED_MAX_RAD = EMPTY_CLOSED_MAX_RAD


class GripperCalibration:
    def __init__(self, runtime):
        self.rt = runtime
        self.station = runtime.adapter
        self.samples = {}

    def check(self, side):
        if side not in ('left', 'right') or not self.station.commissioning_path:
            raise ValueError('gripper_commissioning_unavailable')
        if self.rt.closed or self.rt.paused or self.rt.faulted:
            raise ValueError('station_closed_paused_or_faulted')
        # Prevent an empty calibration job from replacing a live task's permit.
        if self.rt.physical_session and self.rt.physical_session.active:
            raise ValueError('end_active_motion_authorization_first')

    def parked(self, side):
        self.check(side)
        return self._stationary_feedback()

    def _stationary_feedback(self):
        first = self.rt.get_robot_state()
        encoders = {name: unit.raw['gripper_rad'] for name, unit in self.station.arms.items()}
        return first, encoders

    def acknowledge(self, side):
        """Called under the operator lease. Acknowledge only a completed calibration timeout."""
        rt = self.rt
        if (side not in ('left', 'right') or not rt.faulted or rt.paused or rt.closed or rt.allow_motion
                or rt.teach_fault or (rt.physical_session and rt.physical_session.active)):
            raise ValueError('no_matching_gripper_calibration_fault')
        operation = 'gripper_calibration_' + side
        relevant = {'dispatch', 'execution', 'outcome_unknown', 'fault_acknowledged',
                    'teach_dispatch', 'teach_held', 'teach_recheck_dispatch', 'teach_recheck_held'}
        with (rt.directory / 'events.jsonl').open() as journal:
            previous, last = {}, {}
            for line in journal:
                event = json.loads(line)
                if event['kind'] in relevant:
                    previous, last = last, event
        job = rt.jobs.get(previous.get('job_id'), {})
        if (previous.get('kind') != 'dispatch' or previous.get('operation') != operation
                or last.get('kind') != 'outcome_unknown' or last.get('operation') != operation
                or last.get('diagnostics', {}).get('cause_message') != 'gripper_calibration_increment_unverified'
                or job.get('operation') != 'calibrate_gripper_jog' or job.get('status') != 'outcome_unknown'
                or job.get('dispatch') != 'sent'):
            raise ValueError('no_matching_gripper_calibration_fault')
        first, before = self._stationary_feedback()
        _, after = self._stationary_feedback()
        def validate(state):
            self.station.check_teach_parked(first, state)
            if any(not GRIPPER_RANGE[0] <= value <= GRIPPER_RANGE[1] for value in after.values()):
                raise ValueError('measured_encoder_outside_sdk_range')
        rt.acknowledge_fault('operator_checked_empty_' + side + '_gripper_after_calibration_timeout', validate=validate)
        return dict(side=side, measured_rad=after, acknowledged=True)

    def jog(self, side, delta_rad=None, target_rad=None):
        if (delta_rad is None) == (target_rad is None):
            raise ValueError('select_gripper_increment_or_target')
        value = delta_rad if target_rad is None else target_rad
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError('finite_gripper_angle_required')
        if delta_rad == 0:
            raise ValueError('finite_nonzero_gripper_increment_required')
        parked, before = self.parked(side)
        target = before[side] + delta_rad if target_rad is None else target_rad
        if not GRIPPER_RANGE[0] <= target <= GRIPPER_RANGE[1]:
            raise ValueError('step_outside_sdk_gripper_range')
        def command():
            return self.station.calibration_increment(side, target, parked, before)

        authorize(self.rt, 'gripper-calibration', 'Operator requested a single empty gripper increment')
        try:
            self.station.check_teach_parked(parked, self.rt.get_robot_state())
            result = self.rt._dispatch('gripper_calibration_' + side, command)
        finally:
            revoke(self.rt)
        if result.ok:
            for entry in self.samples.get(side, {}).values():
                if result.diagnostics['target_rad'] != entry.get('command_rad', entry['first_rad']):
                    entry['departed'] = True
        return result

    def record(self, side, endpoint):
        if endpoint not in ('open_rad', 'closed_rad'):
            raise ValueError('unknown_gripper_endpoint')
        _, first = self.parked(side)
        _, last = self.parked(side)
        value = (first[side] + last[side]) / 2.
        if not GRIPPER_RANGE[0] <= value <= GRIPPER_RANGE[1]:
            raise ValueError('measured_endpoint_outside_sdk_range')
        if endpoint == 'closed_rad':
            return self._record_closed(side, value)
        entries = self.samples.setdefault(side, {})
        entry = entries.get(endpoint)
        if entry is None:
            entries[endpoint] = dict(first_rad=value, departed=False)
        elif not entry['departed']:
            raise ValueError('revisit_same_endpoint_after_moving_away')
        else:
            entry['repeat_rad'] = value
        self.rt.event('gripper_endpoint_measurement', side=side, endpoint=endpoint, sample=value)
        if len(entries) == 2 and all('repeat_rad' in row for row in entries.values()):
            opened = (entries['open_rad']['first_rad'] + entries['open_rad']['repeat_rad']) / 2
            return self._save_closed(side, opened, entries['closed_rad'])
        return dict(side=side, calibrated=False, readings=entries)

    def _record_closed(self, side, measured):
        execution = self.rt.last_execution or {}
        receipt = execution.get('diagnostics', {})
        if not execution.get('result_ok') or receipt.get('side') != side:
            raise ValueError('test_selected_closed_target_before_recording')
        target = receipt.get('target_rad')
        if target is None or not 0 <= target <= CLOSED_MAX_RAD or measured > CLOSED_MAX_RAD:
            raise ValueError('empty_closed_position_must_be_at_most_0_1_rad')
        entries = self.samples.setdefault(side, {})
        entry = entries.get('closed_rad')
        if entry is None:
            entries['closed_rad'] = dict(command_rad=target, first_rad=measured, departed=False)
        else:
            if not entry['departed'] or entry['command_rad'] != target:
                raise ValueError('reopen_then_test_the_same_closed_target')
            entry['repeat_rad'] = measured
        self.rt.event('gripper_endpoint_measurement', side=side, endpoint='closed_rad',
                      sample=measured, command_rad=target, maximum_measured_rad=CLOSED_MAX_RAD)
        entry = entries['closed_rad']
        if 'repeat_rad' not in entry:
            return dict(side=side, calibrated=False, readings=entries)
        station = self.station
        opened = station.commissioning['grippers'].get(side, {}).get('open_rad')
        if 'open_rad' in entries and 'repeat_rad' in entries['open_rad']:
            opened = (entries['open_rad']['first_rad'] + entries['open_rad']['repeat_rad']) / 2
        if opened is None:
            return dict(side=side, calibrated=False, readings=entries, needs='open_endpoint')
        return self._save_closed(side, opened, entry)

    def _save_closed(self, side, opened, entry):
        station = self.station
        values = dict(open_rad=opened, closed_rad=entry['command_rad'])
        if opened == values['closed_rad']:
            raise ValueError('gripper_endpoints_too_close')
        validation = dict(station.commissioning.get('gripper_validation', {}))
        sides = dict(validation.get('sides', {}))
        sides[side] = dict(status='empty_closed_verified', maximum_measured_rad=CLOSED_MAX_RAD,
                           command_rad=entry['command_rad'],
                           measured_rad=[entry['first_rad'], entry['repeat_rad']],
                           source='operator_recorded_after_two_commands', timestamp_s=time.time())
        validation.update(status='independent_closed_calibration', sides=sides)
        saved = dict(station.commissioning,
                     grippers={**station.commissioning['grippers'], side: values},
                     gripper_validation=validation)
        write_json(station.commissioning_path, saved)
        station.commissioning = saved
        station.arms[side].gripper = values['closed_rad'], values['open_rad']
        station.unsupported_operations.difference_update({'open_gripper','close_gripper','set_gripper','set_grippers'})
        self.rt.plans.clear()
        self.rt.event('gripper_calibrated', side=side, ports=station.ports,
                      readings=self.samples[side], endpoints=values, validation=sides[side])
        del self.samples[side]
        return dict(side=side, calibrated=True, endpoints=values, validation=sides[side])
