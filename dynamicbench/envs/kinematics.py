"""Current-state seeded bounded IK from the same URDF as the USD robot.

No planner execution or joint state writes. Live FK agreement is checked by
ConveyorCubeTask before any episode is permitted.
"""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


class URDFArmIK:
    def __init__(self, path, base_position, base_rotation, joint_names, end_link=None):
        root = ET.parse(path).getroot()
        self.base = np.eye(4)
        self.base[:3, :3] = base_rotation
        self.base[:3, 3] = base_position
        self.joints = []
        self.lower, self.upper = [], []
        self.chain = []
        if end_link is None:
            chain = [root.find(f"joint[@name='{name}']") for name in joint_names]
        else:
            parents = {j.find("child").attrib["link"]: j for j in root.findall("joint")}
            chain, link, seen = [], end_link, set()
            while link in parents:
                if link in seen:
                    raise ValueError("cyclic URDF chain")
                seen.add(link)
                joint = parents[link]
                chain.insert(0, joint)
                link = joint.find("parent").attrib["link"]
            moving = [j.attrib["name"] for j in chain if j.attrib["type"] != "fixed"]
            if moving != list(joint_names):
                raise ValueError(f"URDF chain does not match configured arm joints: {moving}")
        for j in chain:
            if j is None:
                raise ValueError("missing URDF joint")
            name = j.attrib["name"]
            if j.attrib["type"] not in {"revolute", "fixed"}:
                raise ValueError(f"unsupported/missing joint: {name}")
            origin = j.find("origin")
            m = np.eye(4)
            attrs = {} if origin is None else origin.attrib
            m[:3, 3] = np.fromstring(attrs.get("xyz", "0 0 0"), sep=" ")
            m[:3, :3] = Rotation.from_euler("xyz", np.fromstring(attrs.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            if j.attrib["type"] == "fixed":
                self.chain.append((m, None, None))
                continue
            axis = np.fromstring(j.find("axis").attrib["xyz"], sep=" ")
            axis = axis / np.linalg.norm(axis)
            self.chain.append((m, axis, len(self.joints)))
            self.joints.append((m, axis))
            self.lower.append(float(j.find("limit").attrib["lower"]))
            self.upper.append(float(j.find("limit").attrib["upper"]))
        self.lower, self.upper = np.asarray(self.lower), np.asarray(self.upper)

    def fk(self, q):
        m = self.base.copy()
        if len(q) != len(self.joints):
            raise ValueError("IK joint dimension mismatch")
        for origin, axis, index in self.chain:
            rot = np.eye(4)
            if index is not None:
                rot[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
            m = m @ origin @ rot
        return m

    def solve(self, position, rotation, current_q, limits=None):
        lower, upper = (self.lower, self.upper) if limits is None else limits
        lower = np.maximum(lower, self.lower)
        upper = np.minimum(upper, self.upper)

        def residual(q):
            m = self.fk(q)
            return np.r_[m[:3, 3] - position, 0.15 * Rotation.from_matrix(rotation @ m[:3, :3].T).as_rotvec()]

        seed = np.clip(current_q, lower + 1e-7, upper - 1e-7)
        result = least_squares(residual, seed, bounds=(lower, upper), max_nfev=100, ftol=1e-7, xtol=1e-7, gtol=1e-7)
        err = residual(result.x)
        if np.linalg.norm(err[:3]) > 0.003 or np.linalg.norm(err[3:]) > 0.015:
            return None
        return result.x
