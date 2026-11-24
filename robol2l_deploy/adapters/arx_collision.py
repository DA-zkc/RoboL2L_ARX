"""Arm-to-arm collision queries along the nominal joint-space interpolation."""
from hashlib import sha256
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


class PairCollision:
    def __init__(self, config, directory):
        self.required = config.get('required', False)
        if type(self.required) is not bool:
            raise ValueError('collision.required must be boolean')
        self.missing = []
        self.base = {}
        for side in ('left', 'right'):
            value = config.get('base_transforms', {}).get(side)
            if value is None:
                self.missing.append(side + '_base_transform')
                continue
            t = np.asarray(value, dtype=float)
            if (t.shape != (4, 4) or not np.isfinite(t).all() or
                    not np.allclose(t[3], [0, 0, 0, 1]) or
                    not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3)) or
                    not np.isclose(np.linalg.det(t[:3, :3]), 1)):
                raise ValueError(f'{side}: invalid_rigid_base_transform')
            self.base[side] = t
        default = Path(__file__).resolve().parents[3] / 'ARX5_beta/bimanual/models/X5-2023/urdf/X5-2023.urdf'
        urdf = Path(directory) / config.get('urdf', default)
        packages = Path(directory) / config.get('package_dir', urdf.resolve().parents[2])
        self.geometry_digest = None
        if not urdf.is_file():
            self.missing.append('collision_urdf')
            return
        content = urdf.read_bytes()
        model_hash = sha256(content)
        links = ET.fromstring(content).findall('link')
        if any(link.find('collision/geometry') is None for link in links):
            self.missing.append('incomplete_link_collision_geometry')
        for mesh in ET.fromstring(content).findall('.//collision/geometry/mesh'):
            uri = mesh.attrib['filename']
            path = packages / uri[10:] if uri.startswith('package://') else urdf.parent / uri
            if path.is_file():
                model_hash.update(path.read_bytes())
            else:
                self.missing.append(uri)
        self.geometry_digest = model_hash.hexdigest()
        if self.missing:
            return
        import pinocchio as pin
        self.model = pin.buildModelFromUrdf(str(urdf))
        if self.model.nq != 6 or list(self.model.names)[1:] != [f'joint{i}' for i in range(1, 7)]:
            raise ValueError('collision_model_requires_x5_joint_order')
        self.geom = pin.buildGeomFromUrdf(self.model, str(urdf), pin.GeometryType.COLLISION,
                                          package_dirs=[str(packages)])
        if not self.geom.ngeoms:
            raise ValueError('empty_collision_geometry')
        self.placements = {side: (self.model.createData(), self.geom.createData()) for side in self.base}

    def check(self, start, end, *, required=False, paths=None):
        if self.missing:
            if required or self.required:
                raise ValueError('collision_check_unavailable: ' + ', '.join(self.missing))
            return False
        import coal
        import pinocchio as pin
        if paths is None:
            raise ValueError('native_collision_path_required')
        count = max(len(path) for path in paths.values())
        for index in range(count):
            placements = {}
            for side, (data, gd) in self.placements.items():
                joints = np.asarray(paths[side][min(index, len(paths[side])-1)])
                pin.updateGeometryPlacements(self.model, data, self.geom, gd, joints)
                base = pin.SE3(self.base[side])
                placements[side] = [(g, (base * p)) for g, p in zip(self.geom.geometryObjects, gd.oMg)]
            for a, ta in placements['left']:
                for b, tb in placements['right']:
                    if coal.collide(a.geometry, coal.Transform3s(ta.rotation, ta.translation),
                                    b.geometry, coal.Transform3s(tb.rotation, tb.translation),
                                    coal.CollisionRequest(), coal.CollisionResult()):
                        raise ValueError(f'inter_arm_collision: left/{a.name} right/{b.name} native sample {index}')
        return True
