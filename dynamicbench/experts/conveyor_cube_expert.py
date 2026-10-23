"""Privileged feedback oracle. Never exposed as a learned policy observation."""

import numpy as np

from dynamicbench.envs.poses import decode_eef10, encode_eef10


class ConveyorCubeExpert:
    def __init__(self, settings, belt_top):
        self.cfg = settings["expert"]
        self.settings = settings
        self.belt_top = belt_top
        self.phase = "observe_motion"
        self.phase_tick = 0
        self.rotation = np.column_stack([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.offset = np.asarray(settings["robot"]["tcp_offset_link6"])
        self.grasp_xy = None
        self.failure = None

    def change(self, phase):
        self.phase, self.phase_tick = phase, 0

    def action(self, obs):
        self.phase_tick += 1
        pos, rot, grip = decode_eef10(obs["eef10"])
        tcp = pos + rot @ self.offset
        dbg = obs["oracle_debug"]
        cube, velocity = dbg["cube_pose"][:3], dbg["cube_velocity"]
        intercept = cube[:2] + np.clip(velocity[:2], -0.1, 0.1) * self.cfg["intercept_seconds"]
        target = np.r_[intercept, self.belt_top + self.cfg["approach_height"]]
        opening = 1.0
        if self.phase == "observe_motion":
            if self.phase_tick >= self.cfg["observe_ticks"]:
                self.change("pregrasp")
            return obs["eef10"].copy()[:9].tolist() + [1.0]
        if self.phase == "pregrasp":
            if np.linalg.norm(tcp - target) < 0.012 and np.linalg.norm(rot - self.rotation) < 0.08:
                self.change("descend")
        if self.phase in ("descend", "close_gripper"):
            target = np.r_[intercept, self.belt_top + self.settings["cube"]["size"] * 0.55]
            if self.phase == "descend" and np.linalg.norm(tcp - target) < 0.008:
                self.change("close_gripper")
            if self.phase == "close_gripper":
                opening = -1.0
                if self.phase_tick >= self.cfg["close_ticks"] and np.all(dbg["finger_contact_forces"] > 0.01):
                    self.grasp_xy = cube[:2].copy()
                    self.change("lift")
                elif self.phase_tick > 75:
                    self.failure = "no_bilateral_grasp_contact"
        if self.phase == "lift":
            opening = -1.0
            target = np.r_[self.grasp_xy, self.belt_top + self.cfg["lift_height"]]
        speed = self.cfg["descend_speed"] if self.phase == "descend" else self.cfg["max_tcp_speed"]
        delta = target - tcp
        delta *= min(1.0, speed / 30 / max(np.linalg.norm(delta), 1e-9))
        commanded_tcp = tcp + delta
        return encode_eef10(commanded_tcp - self.rotation @ self.offset, self.rotation, opening)
