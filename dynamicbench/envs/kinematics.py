"""Current-state seeded bounded IK from the same URDF as the USD robot.

No planner execution or joint state writes. Live FK agreement is checked by
ConveyorCubeTask before any episode is permitted.
"""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class URDFArmIK:
    def __init__(self, path, base_position, base_rotation, joint_names):
        root = ET.parse(path).getroot()
        self.base = np.eye(4)
        self.base[:3, :3] = base_rotation
        self.base[:3, 3] = base_position
        self.joints = []
        self.lower, self.upper = [], []
        for name in joint_names:
            j = root.find(f"joint[@name='{name}']")
            if j is None or j.attrib['type'] != 'revolute':
                raise ValueError(f'unsupported/missing joint: {name}')
            origin = j.find('origin')
            m = np.eye(4)
            m[:3, 3] = np.fromstring(origin.attrib.get('xyz', '0 0 0'), sep=' ')
            m[:3, :3] = Rotation.from_euler('xyz', np.fromstring(
                origin.attrib.get('rpy', '0 0 0'), sep=' ')).as_matrix()
            axis = np.fromstring(j.find('axis').attrib['xyz'], sep=' ')
            self.joints.append((m, axis))
            self.lower.append(float(j.find('limit').attrib['lower']))
            self.upper.append(float(j.find('limit').attrib['upper']))
        self.lower, self.upper = np.asarray(self.lower), np.asarray(self.upper)

    def fk(self, q):
        m = self.base.copy()
        for (origin, axis), angle in zip(self.joints, q, strict=True):
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            m = m @ origin @ rot
        return m

    def solve(self, position, rotation, current_q, limits=None):
        lower, upper = (self.lower, self.upper) if limits is None else limits
        lower = np.maximum(lower, self.lower)
        upper = np.minimum(upper, self.upper)
        def residual(q):
            m = self.fk(q)
            return np.r_[m[:3, 3] - position,
                         0.15 * Rotation.from_matrix(rotation @ m[:3, :3].T).as_rotvec()]
        seed = np.clip(current_q, lower + 1e-7, upper - 1e-7)
        result = least_squares(residual, seed, bounds=(lower, upper),
                               max_nfev=100, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        err = residual(result.x)
        if np.linalg.norm(err[:3]) > .003 or np.linalg.norm(err[3:]) > .015:
            return None
        return result.x
