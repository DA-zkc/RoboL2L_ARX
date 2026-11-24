"""Physical-station interfaces exercised exclusively with synthetic SDK handles."""
from dataclasses import replace
from pathlib import Path
import json
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch

import numpy as np

from robol2l_deploy.adapters.arx_collision import PairCollision
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.contracts import ArmCommand, RobotAction, MotionStrategy
from robol2l_deploy.runtime import StationRuntime
from test_arx_dual import CONFIG, fixture_station, wrist
from robol2l_deploy.adapters.arx_dual import DualARXStation


class ControlTests(TestCase):
    def setUp(self):
        folder = TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.pair = fixture_station()
        self.rt = StationRuntime(self.pair, self.root, allow_motion=True)
        self.addCleanup(self.rt.close)
        self.api = PublicAPI(self.rt, MockReachTask.context, providers={})
        self.tools = self.api.tools()
        self.left = self.pair.arms['left'].driver.arm
        self.right = self.pair.arms['right'].driver.arm

    def path(self):
        result = self.tools['plan_synchronized_motion']({'left': wrist(.02), 'right': wrist(-.02)})
        self.assertTrue(result.ok, result.error)
        t = result.trajectory
        return replace(t, joint_positions={s: np.vstack([q[0], q[-1]/2, q[-1]])
                                           for s, q in t.joint_positions.items()})

    def test_local_pose_ik_mixed_targets_and_wrong_side(self):
        poses = self.tools['get_robot_state']().end_effector_poses
        for side, pose in poses.items():
            goal = replace(pose, position=pose.position + [.01, 0, 0])
            self.assertTrue(self.tools['solve_ik'](goal, arm=side).ok)
            move = self.tools['move_to_pose'](goal, arm=side)
            self.assertTrue(move.ok, move.error)
            self.assertLess(move.diagnostics['goal_checks'][side]['position_error_m'], 1e-8)
        self.assertTrue(self.tools['plan_synchronized_motion']({'left': poses['left'], 'right': wrist(.02)}).ok)
        self.assertFalse(self.tools['plan_motion'](poses['right'], arm='left').ok)
        self.assertFalse(self.tools['plan_synchronized_motion']({'left': poses['right'], 'right': wrist(0)}).ok)

    def test_policy_tolerance_and_budget(self):
        targets = {'left': wrist(.04), 'right': wrist(-.04)}
        pose = self.tools['get_robot_state']().end_effector_poses['left']
        for name, args, kwargs in [('move_to_joints', [targets['left']], {'arm':'left'}),
                                   ('move_synchronized', [targets], {}), ('move_to_pose', [pose], {'arm':'left'})]:
            result = self.tools[name](*args, max_steps=1, **kwargs)
            self.assertFalse(result.ok)
            self.assertEqual(result.error.details['waypoint_count'], 2)
        self.assertFalse(self.left.commands or self.right.commands)
        native = self.left.set_joint_positions
        def biased(**kwargs):
            accepted = native(**kwargs)
            self.left.q[-1] -= .012
            return accepted
        with patch.object(self.left, 'set_joint_positions', side_effect=biased):
            result = self.tools['move_synchronized'](targets, tolerance=.001)
        self.assertFalse(result.ok)
        self.assertEqual(result.error.message, 'arx_goal_tolerance_not_met')
        self.assertAlmostEqual(result.final_errors['left'], .012)
        self.assertEqual(result.steps_executed, 2)
        self.assertFalse(self.rt.faulted)
        self.assertTrue(self.tools['move_to_joints'](wrist(.02), arm='left').ok)

    def test_preflight_all_waypoints_and_report_partial(self):
        trajectory = self.path()
        bad = {s:v.copy() for s,v in trajectory.joint_positions.items()}
        bad['right'][-1, -1] = 3.
        self.assertFalse(self.tools['execute_trajectory'](replace(trajectory, joint_positions=bad)).ok)
        self.assertFalse(self.left.commands or self.right.commands)
        native = self.right.set_joint_positions
        def reject_second(**kw):
            return native(**kw) if not self.right.commands else False
        with patch.object(self.right, 'set_joint_positions', side_effect=reject_second):
            result = self.tools['execute_trajectory'](trajectory)
        self.assertFalse(result.ok)
        self.assertEqual(result.diagnostics['waypoint'], 2)
        self.assertEqual(result.steps_executed, 2)
        self.assertEqual(result.diagnostics['per_arm']['left']['dispatch'], 'accepted')
        self.assertEqual(result.diagnostics['per_arm']['right']['dispatch'], 'rejected')
        self.assertTrue(self.rt.faulted)

    def test_successful_waypoints_and_pause_boundary(self):
        trajectory = self.path()
        result = self.tools['execute_trajectory'](trajectory)
        self.assertTrue(result.ok, result.error)
        self.assertEqual(result.steps_executed, 3)
        self.assertEqual(len(self.left.commands), 2)
        self.left.q *= 0
        self.right.q *= 0
        with self.rt.program_lease(should_stop=lambda:len(self.right.commands) >= 3):
            result = self.tools['execute_trajectory'](trajectory)
        self.assertFalse(result.ok)
        self.assertEqual(len(self.left.commands), 3)
        self.assertEqual(result.steps_executed, 2)
        self.assertFalse(self.rt.faulted)

    def test_home_alias_and_boundary_normalization(self):
        self.pair.arms['left'].kin.q_min = np.array([-2,0,-2,-2,-2,-2])
        self.left.q[1] = -.001
        state = self.tools['get_robot_state']()
        goal = state.joint_positions['left'].copy()
        goal[-1] = .02
        p = self.tools['plan_motion'](goal, arm='left')
        self.assertTrue(p.ok, p.error)
        self.assertAlmostEqual(p.diagnostics['boundary_adjustments'][0]['measured_rad'], -.001)
        self.assertEqual(p.trajectory.joint_positions[-1,1], 0.)
        goal[1] = -.0012
        self.assertFalse(self.tools['plan_motion'](goal, arm='left').ok)
        self.left.q[1] = goal[1] = -.01
        self.assertTrue(self.tools['plan_motion'](goal, arm='left').ok)
        self.left.q[1] = 0.
        for side, expected in [('secondary',(False,True)),('primary',(True,False)),('both',(True,True)),(None,(True,True))]:
            self.left.q[-1], self.right.q[-1] = .02, -.02
            r=self.tools['go_home'](**({'arm':side} if side else {}))
            self.assertTrue(r.ok,r.error)
            self.assertEqual((self.left.q[-1]==0., self.right.q[-1]==0.),expected)

    def test_step_and_hand_preflight(self):
        embodiment = self.api.environment.embodiment
        commands = {s:ArmCommand('joint_position',wrist(v),embodiment=embodiment)
                    for s,v in [('left',.01),('right',-.01)]}
        for arms in ({'right':commands['right']}, commands):
            step=self.tools['step'](RobotAction(arms))
            self.assertTrue(step.ok,step.error)
            self.assertEqual(set(step.observation.robot_state.arms),set(commands))
        count=len(self.left.commands)
        commands['left']=replace(commands['left'],target=wrist(.8))
        self.assertTrue(self.tools['step'](RobotAction(commands)).ok)
        self.assertEqual(len(self.left.commands),count+1)
        trajectory=self.path()
        hands={s:np.full(3,.5) for s in commands}
        execution=self.tools['execute_trajectory'](replace(trajectory,gripper_positions=hands))
        self.assertFalse(execution.ok)
        self.assertIn('gripper_calibration_required',execution.error.message)
        self.assertEqual(len(self.left.commands),count+1)

    def test_per_side_gripper_calibration_and_waypoint_columns(self):
        config = dict(json.loads(CONFIG.read_text()), cameras={}, extrinsics=None, payload=None, commissioning=None, grasp_geometry=None)
        config['grippers'] = {'left': {'closed_rad': .3, 'open_rad': 3.3}}
        station = DualARXStation(config, directory=self.root)
        for side, unit in station.arms.items():
            unit.driver = self.pair.arms[side].driver
            unit.kin = self.pair.arms[side].kin
        runtime = StationRuntime(station, self.root/'calibrated', allow_motion=True)
        self.addCleanup(runtime.close)
        tools = PublicAPI(runtime, MockReachTask.context, providers={}).tools()
        self.assertEqual(station.describe()['calibrated_grippers'], ['left'])
        def submit(target, duration):
            self.left.gripper = target
            return True
        with patch.object(self.left, 'set_gripper_pos', side_effect=submit) as hand:
            rejected = tools['set_grippers']({'left': .5, 'right': .5})
            self.assertFalse(rejected.ok)
            self.assertIn('right', rejected.error.message)
            hand.assert_not_called()
            self.assertFalse(self.left.commands or self.right.commands)
            result = tools['close_gripper'](arm='left')
            self.assertTrue(result.ok, result.error)
            self.assertEqual(result.diagnostics['grippers']['left']['measured_rad'], .3)
            t = tools['plan_motion'](wrist(.01), arm='left').trajectory
            result = tools['execute_trajectory'](replace(t, gripper_positions=np.array([0., .5])))
            self.assertTrue(result.ok,result.error)
            final = result.diagnostics['waypoint_receipts'][-1]['grippers']['left']
            self.assertAlmostEqual(final['target_rad'], 1.8)
            self.assertAlmostEqual(final['error_rad'], 0.)
            self.assertTrue(tools['open_gripper'](arm='left').ok)
            self.assertAlmostEqual(self.left.gripper, 3.3)
            self.assertFalse(self.right.commands)

    def test_recorded_reset_uses_its_own_snapshot_and_both_hands_can_be_addressed(self):
        home = self.tools['get_robot_state']().joint_positions
        recorded = {'left': wrist(.025), 'right': wrist(-.035)}
        self.assertTrue(self.rt.return_recorded_joints(recorded).ok)
        np.testing.assert_allclose(self.right.q, recorded['right'])
        self.assertTrue(self.tools['go_home']().ok)
        np.testing.assert_allclose(self.left.q, home['left'])
        np.testing.assert_allclose(self.right.q, home['right'])

        config = dict(json.loads(CONFIG.read_text()), cameras={}, extrinsics=None, payload=None, commissioning=None, grasp_geometry=None)
        config['grippers'] = {'left': {'open_rad': 4., 'closed_rad': 0.},
                              'right': {'open_rad': .1, 'closed_rad': 3.1}}
        station = DualARXStation(config, directory=self.root)
        runtime = StationRuntime(station, self.root/'both-hands', allow_motion=True)
        self.addCleanup(runtime.close)
        for side, unit in station.arms.items():
            unit.driver = self.pair.arms[side].driver
            unit.kin = self.pair.arms[side].kin
        def submit(arm):
            def set_position(target, duration):
                arm.gripper = target
                return True
            return set_position
        tools = PublicAPI(runtime, MockReachTask.context, providers={}).tools()
        with patch.object(self.left, 'set_gripper_pos', side_effect=submit(self.left)), \
                patch.object(self.right, 'set_gripper_pos', side_effect=submit(self.right)):
            result = tools['set_grippers']({'left': .25, 'right': 1.})
        self.assertTrue(result.ok, result.error)
        self.assertAlmostEqual(self.left.gripper, 1.)
        self.assertAlmostEqual(self.right.gripper, .1)

    def test_collision_missing_and_intermediate_collision(self):
        required=MotionStrategy(ik_solver='station', pose_planner='station', require_collision_check=True)
        plan=self.tools['plan_motion'](wrist(.04),arm='left',strategy=required)
        self.assertFalse(plan.ok)
        self.assertIn('left_base_transform',plan.error.details['missing'])
        # The fingertip sweeps a circle around joint1. With mirrored bases it
        # meets the stationary right fingertip halfway between two clear ends.
        urdf=self.root/'pair.urdf'
        links=['<link name="base"><collision><geometry><sphere radius=".01"/></geometry></collision></link>']
        parent='base'
        for i in range(1,7):
            x='.2' if i==6 else '0'
            links.append(f'<link name="l{i}"><collision><origin xyz="{x} 0 0"/><geometry><sphere radius=".01"/></geometry></collision></link>'
                         f'<joint name="joint{i}" type="continuous"><parent link="{parent}"/><child link="l{i}"/><axis xyz="0 0 1"/></joint>')
            parent=f'l{i}'
        urdf.write_text(('<robot name="pair">'+''.join(links)+'</robot>').replace('type="continuous"','type="revolute"').replace('<axis','<limit lower="-2" upper="2" effort="1" velocity="1"/><axis'))
        right=np.diag([-1.,-1.,1.,1.]);right[0,3]=.4
        self.pair.collision=PairCollision(dict(urdf=str(urdf),base_transforms={'left':np.eye(4),'right':right}),self.root)
        self.left.q[0]=-.4
        clear=self.tools['plan_motion'](self.left.q.copy(),arm='left',strategy=required)
        self.assertTrue(clear.ok, clear.error)
        self.assertTrue(clear.trajectory.collision_aware)
        target=self.left.q.copy();target[0]=.4
        blocked=self.tools['plan_motion'](target,arm='left',strategy=required)
        self.assertFalse(blocked.ok)
        self.assertIn('inter_arm_collision',blocked.error.message)
        self.assertFalse(self.left.commands or self.right.commands)
