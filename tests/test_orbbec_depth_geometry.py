"""No device handles: synthetic frames exercise the installed alignment SDK."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from robol2l_deploy.cameras import Frame, rectify
from robol2l_deploy.depth_capture import DepthCapture


class RectificationTests(unittest.TestCase):
    def setUp(self):
        y, x = np.indices((48, 64))
        self.frame = Frame(np.stack([x*3, y*4, x+y], axis=-1).astype(np.uint8),
            (.3+x*.001+y*.002).astype(np.float32),
            dict(fx=35., fy=35., ppx=32., ppy=24., coeffs=[.2,0,0,0,0],
                 model='distortion.brown_conrady'), 1.)

    def test_ideal_color_depth_is_not_remapped_again(self):
        frame = Frame(self.frame.rgb, self.frame.depth_m,
            dict(self.frame.metadata, depth_pixel_geometry='undistorted_color'), 1.)
        rgb, depth, _ = rectify(frame)
        np.testing.assert_array_equal(depth, frame.depth_m)
        self.assertFalse(np.shares_memory(depth, frame.depth_m))
        self.assertFalse(np.array_equal(rgb, frame.rgb))

    def test_legacy_distorted_color_depth_still_remaps(self):
        _, old, _ = rectify(self.frame)
        self.assertFalse(np.array_equal(old, self.frame.depth_m))
        explicit = Frame(self.frame.rgb, self.frame.depth_m,
            dict(self.frame.metadata, depth_pixel_geometry='distorted_color'), 1.)
        np.testing.assert_array_equal(old, rectify(explicit)[1])

    def test_unknown_geometry_and_shape_rejected(self):
        with self.assertRaisesRegex(ValueError, 'pixel_geometry'):
            rectify(Frame(self.frame.rgb, self.frame.depth_m,
                dict(self.frame.metadata, depth_pixel_geometry='native_depth'), 1.))
        with self.assertRaisesRegex(ValueError, 'dimensions'):
            rectify(Frame(self.frame.rgb, self.frame.depth_m[:10], self.frame.metadata, 1.))


class NativeAlignTests(unittest.TestCase):
    def setUp(self):
        try:
            import pyorbbecsdk as ob
        except ImportError:
            self.skipTest('native camera SDK not installed; no devices required')
        self.ob = ob
        self.color = ob.Frame.create_video_frame(ob.OBFrameType.COLOR_FRAME, ob.OBFormat.RGB, 64, 48)
        self.depth = ob.Frame.create_video_frame(ob.OBFrameType.DEPTH_FRAME, ob.OBFormat.Y16, 64, 48)
        k = ob.OBCameraIntrinsic()
        for n, v in dict(fx=35.,fy=35.,cx=32.,cy=24.,width=64,height=48).items():setattr(k,n,v)
        distorted = ob.OBCameraDistortion();distorted.model=ob.OBCameraDistortionModel.BROWN_CONRADY
        distorted.k1=.2
        zero=ob.OBCameraDistortion();zero.model=ob.OBCameraDistortionModel.NONE
        cp=self.color.get_stream_profile().as_video_stream_profile()
        dp=self.depth.get_stream_profile().as_video_stream_profile()
        cp.set_intrinsic(k);cp.set_distortion(distorted);dp.set_intrinsic(k);dp.set_distortion(zero)
        extrinsic=ob.OBExtrinsic();extrinsic.rot=np.eye(3,dtype=np.float32);extrinsic.transform=np.zeros(3,np.float32)
        dp.bind_extrinsic_to(cp,extrinsic)
        self.raw=np.full((48,64),400,np.uint16);self.raw[:, :10]=4000
        self.depth.update_data(self.raw.tobytes());self.color.update_data(bytes(64*48*3))
        ob.Frame.set_frame_device_timestamp_us(self.depth,123456)
        ob.Frame.set_frame_device_timestamp_us(self.color,123456)
        self.pair=ob.Frame.create_frame_set();self.pair.push_frame(self.color);self.pair.push_frame(self.depth)
        self.align=ob.AlignFilter(ob.OBStreamType.COLOR_STREAM)
        self.aligned=self.align.process(self.pair).as_frame_set()
        depth_frame=self.aligned.get_depth_frame()
        meters=np.frombuffer(depth_frame.get_data(),np.uint16).reshape(48,64).astype(np.float32)*.001
        self.frame=Frame(np.zeros((48,64,3),np.uint8),meters,
            dict(fx=35.,fy=35.,ppx=32.,ppy=24.,coeffs=[.2,0,0,0,0],model='distortion.brown_conrady',
                 depth_pixel_geometry='undistorted_color'),1.)

    def test_native_default_has_zero_output_distortion(self):
        self.assertEqual(self.align.get_config_value('TargetDistortion'),0)
        d=self.aligned.get_depth_frame().get_stream_profile().as_video_stream_profile().get_distortion()
        self.assertEqual([getattr(d,n) for n in ('k1','k2','p1','p2','k3','k4','k5','k6')],[0]*8)
        np.testing.assert_array_equal(rectify(self.frame)[1],self.frame.depth_m)
        legacy=Frame(self.frame.rgb,self.frame.depth_m,{k:v for k,v in self.frame.metadata.items()
            if k!='depth_pixel_geometry'},1.)
        self.assertGreater(np.count_nonzero(rectify(legacy)[1]!=self.frame.depth_m),0)

    def source_fixture(self):
        from robol2l_deploy.adapters.orbbec_pair import OrbbecSource
        source=OrbbecSource('fixture', None)
        cp=self.color.get_stream_profile().as_video_stream_profile()
        source.k=cp.get_intrinsic();source.distortion=cp.get_distortion()
        encoded=cv2.imencode('.jpg',np.zeros((48,64,3),np.uint8))[1]
        color=SimpleNamespace(get_data=lambda:encoded,get_timestamp_us=lambda:123456,
            get_system_timestamp_us=lambda:123456,get_index=lambda:7)
        frame_set=SimpleNamespace(get_color_frame=lambda:color,get_depth_frame=self.aligned.get_depth_frame)
        source.pipe=SimpleNamespace(wait_for_frames=lambda timeout:self.pair)
        source.align=SimpleNamespace(process=lambda pair:SimpleNamespace(as_frame_set=lambda:frame_set))
        return source

    def test_source_marks_actual_aligned_depth_and_rectifies_rgb_only(self):
        source=self.source_fixture()
        frame=source.read()
        self.assertEqual(frame.metadata['depth_pixel_geometry'],'undistorted_color')
        np.testing.assert_array_equal(rectify(frame)[1],frame.depth_m)
        np.testing.assert_array_equal(frame.depth_m,self.frame.depth_m)

    def test_sdk_frameset_is_not_rejected_by_project_timestamp_gap(self):
        source = self.source_fixture()
        frames = source.align.process(self.pair).as_frame_set()
        frames.get_color_frame().get_timestamp_us = lambda: 223456
        frame = source.read()
        self.assertAlmostEqual(frame.metadata['color_timestamp_ms'] - frame.metadata['depth_timestamp_ms'], 100.)

    def test_no_project_frame_deadline_and_explicit_shutdown_unblocks_reader(self):
        source = self.source_fixture()
        source.pipe.wait_for_frames = Mock(side_effect=[None, None, self.pair])
        with patch('robol2l_deploy.adapters.orbbec_pair.time.monotonic', side_effect=[0., 10000., 20000.]):
            frame = source.read()
        self.assertIsNotNone(frame)
        self.assertEqual(source.pipe.wait_for_frames.call_count, 3)
        source.request_stop()
        with self.assertRaisesRegex(RuntimeError, 'camera_closed'):
            source.read()

    def test_source_refuses_mismatched_intrinsics(self):
        source=self.source_fixture();k=source.k;k.fx+=10;source.k=k
        with self.assertRaisesRegex(ValueError,'intrinsics_mismatch'):
            source.read()

    def test_capture_records_same_native_pair_and_all_stages(self):
        with tempfile.TemporaryDirectory() as directory:
            capture=DepthCapture(directory,'fixture')
            capture.directory.mkdir()
            (capture.directory/'request.json').write_text(json.dumps(dict(id='a'*32,serial='fixture',frames=1)))
            self.assertTrue(capture.wanted())
            capture.record(self.pair,self.aligned,self.frame,self.align)
            capture.queue.join()
            self.assertFalse(capture.wanted())
            root=capture.directory/('a'*32)/'0'
            meta=json.loads((root/'metadata.json').read_text())
            data=np.load(root/'stages.npz')
            np.testing.assert_array_equal(data['native_depth_u16'],self.raw)
            np.testing.assert_array_equal(data['depth_rectified_m'],data['aligned_depth_m'])
            self.assertEqual(meta['native_depth']['timestamp_us'],123456)
            self.assertEqual(meta['native_color']['timestamp_us'],123456)
            self.assertEqual(meta['aligned_depth']['timestamp_us'],123456)
            self.assertEqual(meta['alignment']['TargetDistortion'],0)
            self.assertFalse((capture.directory/'error.json').exists())

    def test_diagnostic_failure_never_breaks_camera_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            capture=DepthCapture(directory,'fixture')
            capture.request=dict(id='b'*32,serial='fixture',frames=1)
            capture.record(None,self.aligned,self.frame,self.align)
            self.assertEqual(capture.queued,1)
            self.assertTrue((capture.directory/'error.json').exists())

    def test_requests_are_bounded_and_cannot_choose_output_paths(self):
        with tempfile.TemporaryDirectory() as directory:
            capture=DepthCapture(directory,'fixture');capture.directory.mkdir()
            for request in (dict(id='../elsewhere',serial='fixture',frames=1),
                            dict(id='a'*32,serial='fixture',frames=9),
                            dict(id='a'*32,serial='other',frames=1),
                            dict(id='a'*32,serial='fixture',frames=True),
                            dict(id='a'*32,serial='fixture',frames=1,path='/tmp/elsewhere')):
                (capture.directory/'request.json').write_text(json.dumps(request))
                self.assertFalse(capture.wanted())


if __name__=='__main__':unittest.main()
