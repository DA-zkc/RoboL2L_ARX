"""Offline rigid payload composition; never opens a CAN device or changes SDK gains."""
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from cap_harness.geometry import matrix_to_pose


def file_sha256(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def inertial(mass, com, inertia):
    mass = float(mass)
    com, inertia = np.asarray(com, dtype=float), np.asarray(inertia, dtype=float)
    if (not np.isfinite(mass) or mass <= 0 or com.shape != (3,) or inertia.shape != (3, 3)
            or not np.isfinite(com).all() or not np.isfinite(inertia).all()
            or not np.allclose(inertia, inertia.T, atol=1e-12, rtol=0)):
        raise ValueError('invalid_payload_inertial')
    eigenvalues = np.linalg.eigvalsh(inertia)
    if eigenvalues[0] <= 0 or eigenvalues[-1] > sum(eigenvalues[:2]) + 1e-12:
        raise ValueError('nonphysical_payload_inertia')
    return mass, com, inertia


def combine(parts):
    """COM and inertia about that COM, all expressed in the same link frame."""
    parts = [inertial(*part) for part in parts]
    mass = sum(part[0] for part in parts)
    com = sum(m * c for m, c, _ in parts) / mass
    inertia = np.zeros((3, 3))
    for m, c, tensor in parts:
        offset = c - com
        inertia += tensor + m * (np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset))
    return inertial(mass, com, inertia)


def read_link6(root):
    node = root.find("link[@name='link6']/inertial")
    origin, tensor = node.find('origin'), node.find('inertia').attrib
    rotation = Rotation.from_euler('xyz', [float(x) for x in origin.get('rpy', '0 0 0').split()]).as_matrix()
    xx, xy, xz, yy, yz, zz = [float(tensor[key]) for key in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz')]
    return inertial(node.find('mass').get('value'), [float(x) for x in origin.get('xyz').split()],
                    rotation @ np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]]) @ rotation.T)


def compose_urdf(base_bytes, spec, camera):
    """Only link6's inertial changes; the SDK model already includes the official gripper."""
    root = ET.fromstring(base_bytes)
    dims = np.asarray(camera['box_dimensions_m'], dtype=float)
    if dims.shape != (3,) or not np.isfinite(dims).all() or np.any(dims <= 0):
        raise ValueError('invalid_camera_box_dimensions')
    mass = float(camera['mass_kg'])
    tensor = np.diag(mass / 12 * (np.sum(dims ** 2) - dims ** 2))
    _, com, tensor = inertial(mass, camera['nominal_com_optical_m'], tensor)
    transform = np.asarray(spec['T_link6_from_camera_optical'], dtype=float)
    matrix_to_pose(transform, frame='link6')
    rotation = transform[:3, :3]
    parts = [read_link6(root), (mass, rotation @ com + transform[:3, 3], rotation @ tensor @ rotation.T)]
    # Measured/official bracket data can be supplied later, in native link6 coordinates.
    for component in spec.get('additional_components', []):
        if not component.get('source'):
            raise ValueError('payload_component_requires_source')
        parts.append(inertial(component['mass_kg'], component['com_link6_m'], component['inertia_com_link6_kg_m2']))
    total, center, inertia = combine(parts)
    node = root.find("link[@name='link6']/inertial")
    node.find('origin').set('xyz', ' '.join(format(x, '.17g') for x in center))
    node.find('origin').set('rpy', '0 0 0')
    node.find('mass').set('value', format(total, '.17g'))
    for key, i, j in (('ixx', 0, 0), ('ixy', 0, 1), ('ixz', 0, 2), ('iyy', 1, 1), ('iyz', 1, 2), ('izz', 2, 2)):
        node.find('inertia').set(key, format(inertia[i, j], '.17g'))
    data = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    return data, dict(link6_mass_kg=total, com_link6_m=center.tolist(), inertia_com_link6_kg_m2=inertia.tolist(),
                      camera_com_link6_m=parts[1][1].tolist())


class PayloadModels:
    def __init__(self, path, ports, cameras, extrinsics):
        self.paths, self.hashes, self.record = {}, {}, {}
        if path is None:
            return
        path = Path(path).resolve()
        record = json.loads(path.read_text())
        if (record.get('schema') != 1 or record.get('ports') != ports
                or set(record.get('sides', {})) != {'left', 'right'}):
            raise ValueError('payload_station_binding_changed')
        base = path.parent / record['base_urdf']['path']
        if file_sha256(base) != record['base_urdf']['sha256']:
            raise ValueError('payload_base_urdf_changed')
        for side, spec in record['sides'].items():
            camera_name = side + '_wrist'
            name = f'T_{side}_flange_from_{camera_name}_optical'
            if (spec['camera_serial'] != cameras.get(camera_name) or name not in extrinsics
                    or not np.array_equal(spec['T_link6_from_camera_optical'], extrinsics[name])):
                raise ValueError('payload_camera_mount_binding_changed: ' + side)
            expected, report = compose_urdf(base.read_bytes(), spec, record['camera'])
            model = (path.parent / spec['urdf_path']).resolve()
            if file_sha256(model) != spec['sha256'] or model.read_bytes() != expected:
                raise ValueError('payload_generated_urdf_changed: ' + side)
            self.paths[side], self.hashes[side] = str(model), spec['sha256']
            spec['computed'] = report
        self.record = record

    def describe(self):
        return deepcopy(self.record) if self.record else dict(status='sdk_default')
