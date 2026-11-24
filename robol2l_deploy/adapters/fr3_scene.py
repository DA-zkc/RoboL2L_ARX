"""FR3 collision geometry for an observed scene and an attached payload."""
import numpy as np
from scipy.spatial.transform import Rotation
from franka_cap.planning import CollisionScene, Kinematics, contact_record


class PayloadCollisionScene(CollisionScene):
    def __init__(self, calibration, obstacles, manipulation, contact_object=None):
        super().__init__(calibration, obstacles)
        self.manipulation = manipulation
        self.contact_object = contact_object
        self.kin = Kinematics()
        # Follow fixed joints from the anchored URDF root. These links cannot
        # move relative to static observed furniture/mounting hardware.
        self.fixed_links = {-1}
        for link in sorted(i for i in self.links if i >= 0):
            joint = self.pb.getJointInfo(self.robot, link, physicsClientId=self.client)
            if joint[2] == self.pb.JOINT_FIXED and joint[16] in self.fixed_links:
                self.fixed_links.add(link)
        self.bodies = {}
        self.table = None
        scene = manipulation.scene
        if scene is None:
            return
        try:
            pb = self.pb
            shape = pb.createCollisionShape(pb.GEOM_PLANE, planeNormal=scene.support_normal, physicsClientId=self.client)
            self.table = pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                basePosition=scene.support_point_m, physicsClientId=self.client)
            for box in scene.boxes:
                shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=np.asarray(box.size_m)/2, physicsClientId=self.client)
                pose = box.pose.as_matrix()
                self.bodies[box.name] = pb.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                    basePosition=pose[:3, 3], baseOrientation=Rotation.from_matrix(pose[:3, :3]).as_quat(), physicsClientId=self.client)
            self.names = tuple(self.bodies)
            self.graspable = np.array([box.graspable for box in scene.boxes], dtype=bool)
            self.bounds = np.array([pb.getAABB(body, physicsClientId=self.client)
                                    for body in self.bodies.values()]).reshape(-1, 2, 3)
        except BaseException:
            self.close()
            raise

    def set_state(self, q, width):
        super().set_state(q, width)
        m = self.manipulation
        if m.holding:
            pose = self.kin.forward(q) @ m.ee_to_object
            self.pb.resetBasePositionAndOrientation(self.bodies[m.body.name], pose[:3, 3],
                Rotation.from_matrix(pose[:3, :3]).as_quat(), physicsClientId=self.client)
            self.bounds[self.names.index(m.body.name)] = self.pb.getAABB(
                self.bodies[m.body.name], physicsClientId=self.client)

    def evidence(self):
        scene = self.manipulation.scene
        return dict(super().evidence(), obstacle_geometry='RGB-D support plane and point envelopes' if scene else 'configured boxes',
            observation_id=scene.observation_id if scene else None,
            calibration_binding=scene.calibration_binding if scene else None)

    def nearby(self, body, distance, link=-1):
        """Conservative AABB lower bound; exact Bullet queries decide contact.

        Negative thresholds still retain every overlapping pair. A small outward
        tolerance avoids dropping boundary contacts through floating point error.
        """
        lo, hi = self.pb.getAABB(body, linkIndex=link, physicsClientId=self.client)
        gap = np.maximum(np.maximum(self.bounds[:, 0] - hi, lo - self.bounds[:, 1]), 0.)
        return np.flatnonzero(np.sum(gap * gap, axis=1) <= (np.maximum(distance, 0.) + 1e-9) ** 2)

    def violations(self, margin=0.):
        hits = super().violations(margin)
        if self.table is None:
            return hits
        pb, m = self.pb, self.manipulation
        held = m.body.name if m.holding else None
        def check(a, b, first, second, distance, link=-1):
            points = pb.getClosestPoints(a, b, distance, linkIndexA=link, physicsClientId=self.client)
            if points:
                hit = contact_record(points, first, second)
                if second in self.bodies:
                    index = self.names.index(second)
                    hit['obstacle_bounds_m'] = self.bounds[index].tolist()
                hits.append(hit)
        clearance = np.full(len(self.names), margin)
        for link in self.shapes:
            if self.links[link] != 'fr3_link0':
                check(self.robot, self.table, self.links[link], 'support', margin, link)
            for index in self.nearby(self.robot, clearance, link):
                name = self.names[index]
                if link in self.fixed_links and name != held:
                    continue  # Static/static pair; attached payloads still move.
                if name in (held, self.contact_object) and link in self.fingers:
                    continue
                check(self.robot, self.bodies[name], self.links[link], name, clearance[index], link)
        check(self.camera, self.table, 'wrist_camera', 'support', margin)
        for index in self.nearby(self.camera, margin):
            name = self.names[index]
            check(self.camera, self.bodies[name], 'wrist_camera', name, margin)
        if held:
            # Permit touching a support at lift/place endpoints, never penetration.
            check(self.bodies[held], self.table, 'held_object', 'support', 0.)
            for index in self.nearby(self.bodies[held], 0.):
                name = self.names[index]
                if name != held:
                    check(self.bodies[held], self.bodies[name], 'held_object', name, 0.)
            for name, body in self.obstacles:
                check(self.bodies[held], body, 'held_object', name, margin)
        return hits
