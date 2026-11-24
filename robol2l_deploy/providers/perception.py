"""Configure upstream perception clients and persist their actual evidence."""
from dataclasses import replace
from pathlib import Path
import json
import os
from uuid import uuid4
import cv2
import numpy as np
import requests
from cap_harness.providers.sam3.client import Sam3Provider
from cap_harness.providers.graspnet.client import ContactGraspNetProvider
from ..runtime import public
from ..contracts import PointCloud, SegmentationSet
from ..errors import ApiError, ErrorCode
from cap_harness.geometry import matrix_to_pose

DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / 'config/perception.json'


def configuration():
    path = Path(os.environ.get('ROBOL2L_PERCEPTION_CONFIG', DEFAULT_CONFIG))
    return json.loads(path.read_text()) if path.exists() else {}


class SegmentationEvidence:
    def __init__(self, client, environment):
        self.client, self.environment = client, environment

    def _call(self, operation, camera, prompt, **kwargs):
        result = getattr(self.client, operation)(camera, prompt, **kwargs)
        env = self.environment
        raw_candidates = [{'score':s.score,'box_xyxy':public(s.box_xyxy)} for s in result.segmentations]
        if result.ok:
            result = replace(result, segmentations=tuple(replace(s, observation_id=env.observation_id) for s in result.segmentations),
                diagnostics=dict(result.diagnostics, observation_id=env.observation_id))
        directory = env.runtime.directory/'perception'/uuid4().hex
        directory.mkdir(parents=True)
        masks = [s.mask for s in result.segmentations] if result.ok else []
        np.savez_compressed(directory/'masks.npz', **{f'mask_{i}':m for i,m in enumerate(masks)})
        overlay = camera.rgb.copy()
        for mask in masks:
            overlay[mask] = (.55*overlay[mask] + .45*np.array([255,0,255])).astype(np.uint8)
        cv2.imwrite(str(directory/'overlay.png'),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
        record = dict(operation=operation, prompt=public(prompt), ok=result.ok,
            diagnostics=public(result.diagnostics),
            original_query=public(prompt) if operation=='segment_text' else None,
            model_query=result.diagnostics.get('model_query') if operation=='segment_text' else None,
            observation_id=env.observation_id, calibration_binding=env.binding,
            candidates=[{'label':s.label,'score':s.score,'box_xyxy':public(s.box_xyxy)} for s in result.segmentations],
            error=public(result.error), raw_candidates=raw_candidates, model='sam3', synthetic=env.runtime.adapter.synthetic)
        (directory/'result.json').write_text(json.dumps(record,allow_nan=False))
        env.runtime.event('perception', artifact=str(directory.relative_to(env.runtime.directory)), **record)
        return result

    def segment_text(self, observation, query, *, model_query=None):
        return self._call('segment_text', observation, query, **({'model_query':model_query} if model_query is not None else {}))

    def segment_points(self, observation, points_px, *, point_labels=None):
        return self._call('segment_points', observation, points_px, point_labels=point_labels)


class GraspEvidence:
    def __init__(self, client, environment):
        self.client, self.environment = client, environment

    def generate_grasps(self, point_cloud, **kwargs):
        env = self.environment
        camera = env.get_observation().cameras[env.perception_camera_name]
        transform = camera.camera_pose.as_matrix()
        if point_cloud.frame != camera.camera_pose.frame:
            raise ValueError('grasp cloud must be in the calibrated base frame')
        def camera_cloud(cloud):
            return PointCloud((cloud.points-transform[:3,3])@transform[:3,:3],camera.frame,cloud.colors)
        inputs=dict(kwargs)
        if inputs.get('scene_point_cloud') is not None:inputs['scene_point_cloud']=camera_cloud(inputs['scene_point_cloud'])
        inputs.pop('geometry',None)  # Optional geometry was expressed in base.
        result = self.client.generate_grasps(camera_cloud(point_cloud), **inputs)
        if result.ok:
            converted=[]
            for grasp in result.grasps:
                metadata=dict(grasp.metadata)
                if 'contact_point' in metadata:
                    metadata['contact_point']=(transform[:3,:3]@np.asarray(metadata['contact_point'])+transform[:3,3]).tolist()
                converted.append(replace(grasp,pose=matrix_to_pose(transform@grasp.pose.as_matrix(),frame=point_cloud.frame),metadata=metadata))
            result=replace(result,grasps=tuple(converted),diagnostics=dict(result.diagnostics,frame=point_cloud.frame,inference_frame=camera.frame))
        if result.ok:
            result = replace(result, grasps=tuple(replace(g, metadata=dict(g.metadata,
                observation_id=env.observation_id, calibration_binding=env.binding,
                width_source='network_predicted_opening_not_contact_measurement')) for g in result.grasps))
        directory = env.runtime.directory/'perception'/uuid4().hex
        directory.mkdir(parents=True)
        scene = kwargs.get('scene_point_cloud')
        np.savez_compressed(directory/'clouds.npz', target=point_cloud.points,
            scene=scene.points if scene is not None else point_cloud.points)
        record = dict(operation='generate_grasps', observation_id=env.observation_id,
            calibration_binding=env.binding, model='contact-graspnet', frame=point_cloud.frame,
            synthetic=env.runtime.adapter.synthetic, result=public(result))
        (directory/'result.json').write_text(json.dumps(record,allow_nan=False))
        env.runtime.event('perception',artifact=str(directory.relative_to(env.runtime.directory)),
            operation='generate_grasps',observation_id=env.observation_id,ok=result.ok)
        return result


def configured_providers(environment):
    config = configuration()
    if not config.get('enabled', False):
        return {}
    arx = environment.embodiment in ('arx_x5_2023', 'arx_x5_2023_dual')
    if arx and not environment.runtime.adapter.cameras:
        return {}
    # These endpoints are explicitly local services, never an ambient HTTP proxy.
    session = requests.Session();session.trust_env = False
    sam = Sam3Provider(config['sam3_url'], session=session, max_retries=0,
                      text_threshold=config.get('sam3_text_confidence_threshold', 0.5))
    if arx:
        return {'segmentation':SegmentationEvidence(sam,environment)}
    grasp = ContactGraspNetProvider(config['graspnet_url'], session=session, max_retries=0,
        grasp_to_tcp=np.array(config['grasp_to_tcp']), require_predicted_width=True,
        maximum_width_m=config['maximum_width_m'], service_max_retries=0)
    return {'segmentation':SegmentationEvidence(sam,environment),
            'grasps':{'contact-graspnet':GraspEvidence(grasp,environment)}}
