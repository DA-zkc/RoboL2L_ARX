"""Reproduce a nominal grasp frame from pinned official X5-2023 jaw surfaces.

This defines an application TCP; ARX's URDF does not declare a TCP itself.
No hardware, network, motor-angle conversion, or arm kinematic replacement.
"""
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


MODEL = Path(__file__).resolve().parents[1] / 'models' / 'arx-x5-2023-gripper'
COMMIT = 'a45e8dbb4b1844ae7aef62d16182e5f1724da6d3'
SOURCE = f'https://github.com/ARXroboticsX/ARX_Model/tree/{COMMIT}/X5/X5-2023'
HASHES = {
    'X5.urdf': 'f65bb597b75f4acf54f08edf9989cca42a4c196fe138d7e975e48b140526ec87',
    'meshes/link7.STL': '257656e27b47bfc099c502463599680bc94b68b1d58f6c853ea20327d5f7964a',
    'meshes/link8.STL': '0b9bf8d8ec1b12b684c8e45ac68e8bbfb8ef3c2323bbe90a4defbfe530e0a944',
}


def official_gripper_record(directory=MODEL):
    """Area-centroid midpoint of the two distal inward-facing contact pads.

    The pinned meshes' pads are planar trapezoids, not the whole jaw's bounds,
    inertial COM, jaw origin, or tip. Equal opposite prismatic motion preserves
    this midpoint. +Z TCP = +X link6; +X TCP = +Y link6.
    """
    blobs = {}
    for name, expected in HASHES.items():
        blobs[name] = (Path(directory) / name).read_bytes()
        if hashlib.sha256(blobs[name]).hexdigest() != expected:
            raise ValueError(f'official_gripper_model_hash_mismatch: {name}')
    urdf = ET.fromstring(blobs['X5.urdf'])
    pads = []
    for number, sign in [(7, 1), (8, -1)]:
        joint = urdf.find(f"joint[@name='joint{number}']")
        origin = np.fromstring(joint.find('origin').get('xyz'), sep=' ')
        if (joint.get('type') != 'prismatic' or joint.find('parent').get('link') != 'link6'
                or np.any(np.fromstring(joint.find('origin').get('rpy'), sep=' '))
                or not np.array_equal(np.fromstring(joint.find('axis').get('xyz'), sep=' '), [0, sign, 0])):
            raise ValueError('official_gripper_joint_convention_changed')
        raw = blobs[f'meshes/link{number}.STL']
        triangles = np.frombuffer(raw, offset=84, dtype=[
            ('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        vertices = triangles['vertices'].astype(float)
        # Distal pad: inward normal, beyond 50 mm from the jaw mounting origin.
        selected = ((triangles['normal'][:, 1] * sign < -.99999)
                    & (vertices[:, :, 0].min(axis=1) > .05))
        surface = vertices[selected] + origin
        if len(surface) != 2:
            raise ValueError('official_gripper_contact_pad_selection_changed')
        areas = np.linalg.norm(np.cross(surface[:, 1] - surface[:, 0],
                                        surface[:, 2] - surface[:, 0]), axis=1) / 2
        centroid = np.average(surface.mean(axis=1), axis=0, weights=areas)
        pads.append(dict(link=f'link{number}', centroid_link6_m=centroid.tolist(),
                         bounds_link6_m=[surface.min(axis=(0, 1)).tolist(),
                                         surface.max(axis=(0, 1)).tolist()],
                         area_m2=float(areas.sum()),
                         axis_link6=[0, sign, 0],
                         travel_limits_m=[float(joint.find('limit').get(k)) for k in ('lower', 'upper')]))
    tcp = np.eye(4)
    tcp[:3, :3] = [[0, 0, 1], [1, 0, 0], [0, 1, 0]]
    tcp[:3, 3] = np.mean([p['centroid_link6_m'] for p in pads], axis=0)
    record = dict(
        verified=False, model_verified=True, verification='official_model',
        opening_invariant=True, source=SOURCE,
        model=dict(repository='ARXroboticsX/ARX_Model', commit=COMMIT,
                   variant='X5/X5-2023', sha256=HASHES.copy()),
        T_link6_from_grasp_tcp=tcp.tolist(),
        contact_geometry=dict(
            definition='midpoint_of_area_centroids_of_distal_inward_contact_pads',
            pads=pads, nominal_total_jaw_travel_m=.088,
            motor_angle_to_width_calibrated=False,
            note='Nominal CAD geometry; verified=false means no physical TCP acceptance. '
                 'URDF jaw coordinates are metres, not motor radians. 88 mm is summed model '
                 'travel, not a measured opening at the commissioned encoder endpoints. '
                 'Contact pads extend beyond the TCP; TCP is not the forwardmost fingertip.'))
    # Stable JSON across NumPy/platform rounding; precision remains sub-nanometre.
    def rounded(value):
        if isinstance(value, float):
            return round(value, 12)
        if isinstance(value, list):
            return [rounded(v) for v in value]
        if isinstance(value, dict):
            return {k: rounded(v) for k, v in value.items()}
        return value
    return rounded(record)
