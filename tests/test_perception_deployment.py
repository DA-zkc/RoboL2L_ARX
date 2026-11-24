"""Software regression checks; fixtures here are never autonomous evidence."""
import hashlib
import json
from dataclasses import replace
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import numpy as np
import requests

from cap_harness.providers.graspnet.client import ContactGraspNetProvider
from cap_harness.providers.sam3.client import Sam3Provider
from robol2l_deploy.adapters.mock import MockAdapter
from robol2l_deploy.adapters.panda import PandaDriver
from robol2l_deploy.cap.api import PublicAPI
from robol2l_deploy.cap.model import messages
from robol2l_deploy.cap.tasks import MockReachTask
from robol2l_deploy.contracts import Pose, Segmentation, PointCloud, GraspCandidate, GraspSet
from robol2l_deploy.providers.perception import GraspEvidence
from robol2l_deploy.runtime import StationRuntime

ROOT=Path(__file__).resolve().parents[1]

class PerceptionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.adapter=MockAdapter(dof=6,camera_names=('survey',))
        self.runtime=StationRuntime(self.adapter,self.tmp.name)
        self.api=PublicAPI(self.runtime,MockReachTask().context,providers={})
        self.obs=self.api.get_observation()
        self.mask=Segmentation(np.ones((120,160),bool),'fixture',1.,'survey','survey',
                               observation_id=self.api.environment.observation_id)

    def tearDown(self):
        self.runtime.close();self.tmp.cleanup()

    def test_shared_executor_pin_and_contract_identity(self):
        from cap_harness.contracts import Pose as SharedPose
        self.assertIs(Pose,SharedPose)
        manifest=json.loads((ROOT/'cap_harness/UPSTREAM.json').read_text())
        self.assertEqual(hashlib.sha256((ROOT/'cap_harness/runtime.py').read_bytes()).hexdigest(),manifest.get('patched_sha256',{}).get('runtime.py',manifest['original_sha256']['runtime.py']))

    def test_snapshot_reuse_raw_masks_and_retained_masks_are_accepted(self):
        cloud=self.api.tools()['mask_to_point_cloud'](self.mask,'survey','base')
        self.assertEqual(cloud.frame,'base')
        self.assertIs(self.api.environment.get_observation(),self.obs)
        self.assertEqual(self.api.tools()['mask_to_point_cloud'](self.mask.mask,'survey','base').frame,'base')
        self.api.get_observation()
        self.assertEqual(self.api.tools()['mask_to_point_cloud'](self.mask,'survey','base').frame,'base')

    def test_configuration_change_does_not_reinterpret_or_reject_frozen_observation(self):
        with patch.object(self.adapter,'device_binding',return_value='new-calibration'):
            self.assertIs(self.api.environment.get_observation(),self.obs)

    def test_depth_filter_removes_boundary_and_invalid_samples(self):
        camera=self.obs.cameras['survey'];depth=camera.depth_m.copy();depth[50,70]=2.;depth[51,71]=0.
        obs=replace(self.obs,cameras={'survey':replace(camera,depth_m=depth)})
        with patch.object(self.adapter,'synthetic',False):
            filtered=self.api.environment.prepare_projection(self.mask,obs)
        self.assertFalse(filtered.mask[0,0]);self.assertFalse(filtered.mask[50,70]);self.assertFalse(filtered.mask[51,71])
        self.assertTrue(filtered.mask[60,80]);self.assertTrue(self.mask.mask[0,0])

    def test_projection_removes_small_islands_without_selecting_largest_object(self):
        mask=np.zeros((120,160),bool)
        mask[10:16,10:16]=True  # Four pixels after erosion: insufficient support.
        mask[30:50,30:50]=True;mask[70:90,100:120]=True
        segmentation=replace(self.mask,mask=mask)
        with patch.object(self.adapter,'synthetic',False):
            filtered=self.api.environment.prepare_projection(segmentation,self.obs)
        self.assertFalse(filtered.mask[12,12])
        self.assertTrue(filtered.mask[40,40]);self.assertTrue(filtered.mask[80,110])
        self.assertTrue(segmentation.mask[12,12])
        events=[json.loads(line) for line in (self.runtime.directory/'events.jsonl').read_text().splitlines()]
        self.assertEqual(events[-1]['removed_small_component_pixels'],4)
        self.assertEqual(events[-1]['observation_id'],self.api.environment.observation_id)

    def test_oracle_is_not_model_input_and_perception_is_declared(self):
        self.api.providers={'segmentation':object(),'grasps':{'contact-graspnet':object()}}
        context=json.loads(messages(self.api,self.obs,[],{'secret_oracle_token':'hidden'})[1]['content'][0]['text'])
        self.assertNotIn('verification',context)
        self.assertNotIn('secret_oracle_token',json.dumps(context))
        self.assertIn('localize_object',context['api']['capabilities']['supported'])
        self.assertIn('generate_grasps',context['api']['capabilities']['supported'])

    def test_service_unavailable_is_actual_failure(self):
        session=Mock();session.request.side_effect=requests.ConnectionError('offline')
        result=Sam3Provider(session=session,max_retries=0).segment_text(self.obs.cameras['survey'],'object')
        self.assertFalse(result.ok);self.assertEqual(result.error.code.value,'provider_unavailable')
        self.assertFalse(result.segmentations)

    def test_legacy_replay_requires_matching_raw_calibration_and_pixels(self):
        from robol2l_deploy.adapters.replay import match_raw_calibrations
        from robol2l_deploy.cameras import rectify,Frame
        camera=self.obs.cameras['survey'];stamp=int(camera.timestamp_s*1e9)
        folder=Path(self.tmp.name)/'archives'/f'raw-survey-{stamp}';folder.mkdir(parents=True)
        metadata={'calibration_sha256':'verified','fx':100,'fy':100,'ppx':80,'ppy':60,'model':'distortion.none','coeffs':[0]*5}
        (folder/'metadata.json').write_text(json.dumps(metadata))
        np.savez_compressed(folder/'frame.npz',rgb=camera.rgb,depth_m=camera.depth_m)
        with patch('franka_cap.devices.calibration',return_value=({},camera.camera_pose.as_matrix(),'verified')):
            evidence=match_raw_calibrations(self.obs,{}, {},Path(self.tmp.name))
            self.assertEqual(evidence['kind'],'matched_archived_raw_frames')
            metadata['calibration_sha256']='different';(folder/'metadata.json').write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'mismatched_recorded_calibration'):
                match_raw_calibrations(self.obs,{}, {},Path(self.tmp.name))

    def test_width_and_tcp_transform_are_explicit(self):
        transform=np.eye(4);transform[2,3]=.1034
        client=ContactGraspNetProvider(grasp_to_tcp=transform,require_predicted_width=True,maximum_width_m=.08)
        raw=np.eye(4)[None];scores=np.array([.8]);contacts=np.zeros((1,3))
        with self.assertRaisesRegex(ValueError,'omitted'):
            client._decode_candidates(raw,scores,contacts,frame='camera',max_candidates=5)
        candidates=client._decode_candidates(raw,scores,contacts,widths=np.array([.04]),frame='camera',max_candidates=5)
        np.testing.assert_allclose(candidates[0].pose.position,[0,0,.1034])
        self.assertEqual(candidates[0].width_m,.04)
        self.assertFalse(client._decode_candidates(raw,scores,contacts,widths=np.array([.09]),frame='camera',max_candidates=5))

    def test_visual_evaluator_rejects_no_progress_and_mixed_observations(self):
        from robol2l_deploy.cap.tasks import VisualPreparationTask
        folder=Path(self.tmp.name)/'evaluation';folder.mkdir()
        def call(tool,output,args=()):
            return {'kind':'program_call','call':{'tool':tool,'ok':True,'observation_id':'obs',
                    'inputs':{'args':list(args)},'output':output}}
        localization={'geometry':{'pose':{'position':[0,0,0]}},'segmentation':{'observation_id':'obs'}}
        events=[{'kind':'observed_scene','scene':{'observation_id':'obs'},'source':'full_rgbd','manual_regions':False},
                call('localize_object',localization,['green cube']),call('localize_object',localization,['plate']),
                call('generate_grasps',{'grasps':[{'score':.5}]}),
                call('plan_motion',{'diagnostics':{'plan_id':'plan'}})]
        def save():(folder/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events))
        save()
        runtime=SimpleNamespace(directory=folder,plans={'plan':{'trajectory':object()}},
            adapter=SimpleNamespace(preview_xyz=lambda t:[[.1,0,0],[.1,0,0]]))
        self.assertFalse(VisualPreparationTask().verify(runtime)['passed'])
        runtime.adapter.preview_xyz=lambda t:[[.1,0,0],[.09,0,0]]
        self.assertTrue(VisualPreparationTask().verify(runtime)['passed'])
        events[3]['call']['observation_id']='old';save()
        self.assertFalse(VisualPreparationTask().verify(runtime)['passed'])

    def test_camera_frame_inference_returns_base_frame_grasp(self):
        camera=self.obs.cameras['survey']
        pose=Pose([.1,.2,.3],[2**-.5,0,0,2**-.5],'base')
        self.api.environment.observation=replace(self.obs,cameras={'survey':replace(camera,camera_pose=pose)})
        self.api.environment.perception_camera_name='survey'
        class Model:
            def generate_grasps(inner,cloud,**kwargs):
                np.testing.assert_allclose(cloud.points,[[.02,0,0]],atol=1e-10)
                return GraspSet(True,[GraspCandidate(Pose([.02,0,0],[1,0,0,0],'survey'),.9,.04)])
        result=GraspEvidence(Model(),self.api.environment).generate_grasps(PointCloud([[.1,.22,.3]],'base'))
        np.testing.assert_allclose(result.grasps[0].pose.position,[.1,.22,.3],atol=1e-10)
        self.assertEqual(result.grasps[0].pose.frame,'base')
        self.assertEqual(result.diagnostics['frame'],'base')

class PandaLibraryTests(unittest.TestCase):
    def test_binding_exception_preserves_device_diagnostics(self):
        from robol2l_deploy.runtime import OutcomeUnknown
        driver=PandaDriver('never-connect',allow_motion=True,library=SimpleNamespace())
        driver.robot=SimpleNamespace(execute_joint_trajectory=Mock(side_effect=RuntimeError('holder mismatch')))
        driver.connect=Mock();driver.trajectories['test']=object()
        try:
            with self.assertRaises(OutcomeUnknown) as caught:driver.execute('test')
            self.assertEqual(caught.exception.diagnostics['cause_message'],'holder mismatch')
            self.assertEqual(caught.exception.diagnostics['cause_type'],'RuntimeError')
        finally:driver.close()

    def test_actual_library_curve_is_previewed_without_connecting(self):
        try:import panda_py
        except ImportError:self.skipTest('optional compiled panda_py library not installed')
        driver=PandaDriver('never-connect')
        try:
            q=np.array([0.,0.,0.,-1.5,0.,1.5,0.]);goal=q.copy();goal[0]+=.01
            plan=driver.plan([q,goal]);curve=driver.trajectories[plan['token']]
            times=np.arange(len(plan['joint_positions']))*plan['dt_s']
            np.testing.assert_allclose(plan['joint_positions'],[curve.get_joint_positions(float(t)) for t in times],atol=1e-10)
            self.assertIsNone(driver.robot)
            with self.assertRaisesRegex(ValueError,'not_commissioned'):driver.execute(plan['token'])
            self.assertIsNone(driver.robot)
        finally:driver.close()

    def test_connection_failure_releases_owner_lock(self):
        library=SimpleNamespace(Panda=Mock(side_effect=RuntimeError('unreachable')),
                                libfranka=SimpleNamespace(RealtimeConfig=SimpleNamespace(kEnforce=1)))
        owner=Mock();owner.__enter__=Mock();owner.__exit__=Mock()
        driver=PandaDriver('test',library=library)
        with patch('franka_cap.common.hardware_lock',return_value=owner):
            with self.assertRaisesRegex(RuntimeError,'unreachable'):driver.connect()
        owner.__exit__.assert_called_once();self.assertIsNone(driver.robot)

if __name__=='__main__':unittest.main()
