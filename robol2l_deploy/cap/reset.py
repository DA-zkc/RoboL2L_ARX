"""Conservative, per-attempt reset eligibility from Runtime evidence only."""
import numpy as np

JOINT_THRESHOLD_RAD = 0.02
TCP_THRESHOLD_M = 0.005


class ResetMotionAudit:
    def __init__(self, initial, before):
        self.numeric_policy = not before.embodiment.startswith("arx_")
        self.initial = initial
        self.last = before
        self.joint_travel = {a: np.zeros(len(q)) for a, q in before.joint_positions.items()}
        self.command_travel = {a: np.zeros(len(q)) for a, q in before.joint_positions.items()}
        self.max_initial_delta = 0.
        self.max_tcp_delta = 0.
        self.blockers = set()
        self.references = 0
        self.dispatches = 0
        self.receipts = 0
        self.sample(before)

    def sample(self, state):
        try:
            if set(state.joint_positions) != set(self.initial['joint_positions']):
                raise ValueError('arms_changed')
            for arm, q in state.joint_positions.items():
                q = np.asarray(q)
                if not np.isfinite(q).all(): raise ValueError('nonfinite_state')
                self.joint_travel[arm] += np.abs(q - self.last.joint_positions[arm])
                self.max_initial_delta = max(self.max_initial_delta, float(np.max(np.abs(q - self.initial['joint_positions'][arm]))))
                p = state.end_effector_poses[arm].position
                origin = self.initial['end_effector_poses'][arm]['position']
                if not np.isfinite(p).all(): raise ValueError('nonfinite_pose')
                self.max_tcp_delta = max(self.max_tcp_delta, float(np.linalg.norm(p - origin)))
            self.last = state
        except (KeyError, TypeError, ValueError):
            self.blockers.add('incomplete_motion_evidence')

    def event(self, kind, data):
        try:
            if kind == 'trajectory_reference':
                t = data['trajectory']
                self.references += 1
                paths = t.joint_positions if hasattr(t.joint_positions, 'items') else {t.arm: t.joint_positions}
                for arm, path in paths.items():
                    path = np.vstack([t.expected_start.joint_positions[arm], path])
                    if not np.isfinite(path).all(): raise ValueError('nonfinite_path')
                    self.command_travel[arm] += np.abs(np.diff(path, axis=0)).sum(axis=0)
                    self.max_initial_delta = max(self.max_initial_delta, float(np.max(np.abs(path - self.initial['joint_positions'][arm]))))
                if t.gripper_positions is not None: self.blockers.add('trajectory_gripper_command')
            elif kind == 'dispatch':
                self.dispatches += 1
                if data['operation'] != 'execute_trajectory': self.blockers.add('gripper_or_other_motion')
                elif self.dispatches > self.references: self.blockers.add('missing_trajectory')
            elif kind == 'execution':
                self.receipts += 1
                if not data['result'].ok: self.blockers.add('unsuccessful_motion_receipt')
            elif kind == 'outcome_unknown': self.blockers.add('outcome_unknown')
        except (AttributeError, KeyError, TypeError, ValueError):
            self.blockers.add('incomplete_motion_evidence')

    def result(self):
        travel = max([float(np.max(x)) for x in [*self.command_travel.values(), *self.joint_travel.values()]] or [float('inf')])
        if self.dispatches != self.receipts: self.blockers.add('missing_terminal_receipt')
        if self.numeric_policy:
            if travel >= JOINT_THRESHOLD_RAD or self.max_initial_delta >= JOINT_THRESHOLD_RAD:
                self.blockers.add('joint_threshold_exceeded')
            if self.max_tcp_delta >= TCP_THRESHOLD_M:
                self.blockers.add('tcp_threshold_exceeded')
        return dict(eligible=not self.blockers, blockers=sorted(self.blockers),
                    joint_threshold_rad=JOINT_THRESHOLD_RAD if self.numeric_policy else None,
                    tcp_threshold_m=TCP_THRESHOLD_M if self.numeric_policy else None,
                    max_joint_travel_rad=travel, max_initial_joint_delta_rad=self.max_initial_delta,
                    max_initial_tcp_delta_m=self.max_tcp_delta, dispatches=self.dispatches, receipts=self.receipts)
