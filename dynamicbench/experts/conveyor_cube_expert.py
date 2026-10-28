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
        self.lift_xy_mode = self.cfg.get("lift_xy_mode", "track_cube")
        self.lift_tcp_frame = self.cfg.get("lift_tcp_frame", "nominal")
        if self.lift_xy_mode not in ("fixed", "track_cube", "contact_anchor"):
            raise ValueError("unknown lift_xy_mode")
        if self.lift_tcp_frame not in ("nominal", "actual_pinch"):
            raise ValueError("unknown lift_tcp_frame")
        self.grasp_relative_offset = None
        self.contact_anchor = None
        self.contact_rotation = None
        self.contact_command_z = None
        self.lift_goal_z = None
        self.bilateral_ticks = 0
        self.contact_loss_ticks = 0
        self.failure = None

    def change(self, phase):
        self.phase, self.phase_tick = phase, 0

    def _contact_action(self, dbg):
        """Hold a world EEF reference after contact; never chase a pushed cube.

        The cube and finger measurements only confirm the grasp and set the
        lift distance once. They do not move the horizontal command reference.
        """
        bilateral = np.all(dbg["finger_contact_forces"] > self.cfg.get("contact_force_threshold", 1.0))
        self.bilateral_ticks = self.bilateral_ticks + 1 if bilateral else 0
        if self.phase == "grasp_settle":
            if self.bilateral_ticks >= self.cfg.get("stable_contact_ticks", 3):
                cube = dbg["cube_pose"][:3]
                self.grasp_xy = cube[:2].copy()
                self.lift_goal_z = self.contact_anchor[2] + max(
                    0.0, self.belt_top + self.cfg["lift_height"] - cube[2]
                )
                self.change("lift")
            elif self.phase_tick >= self.cfg.get("contact_timeout_ticks", 30):
                self.failure = "no_stable_bilateral_grasp"
        if self.phase == "lift":
            self.contact_loss_ticks = 0 if bilateral else self.contact_loss_ticks + 1
            if self.contact_loss_ticks >= self.cfg.get("stable_contact_ticks", 3):
                self.failure = "lost_bilateral_grasp"
            self.contact_command_z = min(
                self.lift_goal_z, self.contact_command_z + self.cfg["max_tcp_speed"] / 30
            )
        command = self.contact_anchor.copy()
        command[2] = self.contact_command_z
        return encode_eef10(command, self.contact_rotation, -1.0)

    def action(self, obs):
        self.phase_tick += 1
        pos, rot, grip = decode_eef10(obs["eef10"])
        tcp = pos + rot @ self.offset
        offset = self.offset
        dbg = obs["oracle_debug"]
        cube, velocity = dbg["cube_pose"][:3], dbg["cube_velocity"]
        if self.lift_xy_mode == "contact_anchor":
            if self.contact_anchor is not None:
                return self._contact_action(dbg)
            if self.phase == "close_gripper" and np.any(
                dbg["finger_contact_forces"] > self.cfg.get("contact_force_threshold", 1.0)
            ):
                self.contact_anchor = pos.copy()
                self.contact_rotation = rot.copy()
                self.contact_command_z = float(pos[2])
                self.change("grasp_settle")
                return self._contact_action(dbg)
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
                if (
                    self.lift_xy_mode != "contact_anchor"
                    and self.phase_tick >= self.cfg["close_ticks"]
                    and np.all(dbg["finger_contact_forces"] > 0.01)
                ):
                    self.grasp_xy = cube[:2].copy()
                    if self.lift_xy_mode == "track_cube" and self.lift_tcp_frame == "actual_pinch":
                        self.grasp_relative_offset = rot.T @ (cube - pos) - dbg["finger_midpoint_in_eef"]
                    self.change("lift")
                elif self.phase_tick > 75:
                    self.failure = "no_bilateral_grasp_contact"
        if self.phase == "lift":
            opening = -1.0
            if self.lift_xy_mode == "track_cube" and self.lift_tcp_frame == "actual_pinch":
                offset = dbg["finger_midpoint_in_eef"] + self.grasp_relative_offset
                tcp = pos + rot @ offset
            xy = cube[:2] if self.lift_xy_mode == "track_cube" else self.grasp_xy
            target = np.r_[xy, self.belt_top + self.cfg["lift_height"]]
        speed = self.cfg["descend_speed"] if self.phase == "descend" else self.cfg["max_tcp_speed"]
        delta = target - tcp
        delta *= min(1.0, speed / 30 / max(np.linalg.norm(delta), 1e-9))
        commanded_tcp = tcp + delta
        return encode_eef10(commanded_tcp - self.rotation @ offset, self.rotation, opening)
