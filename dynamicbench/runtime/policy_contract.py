"""Explicit model/embodiment declaration, checked before creating the scene.

Factories run inference locally or wrap a model service in its own environment.
No conversion of Piper/LIBERO action vectors is silently assumed.
"""

import numpy as np


class CheckedPolicy:
    def __init__(self, policy, config, *, embodiment="x5"):
        self.validate_config(config, embodiment=embodiment)
        if not callable(getattr(policy, "predict", None)):
            raise ValueError("policy must expose predict(observation, request_tick, horizon)")
        self.policy, self.provenance = policy, dict(config)
        self.supports_rtc_conditioning = bool(getattr(policy, "supports_rtc_conditioning", False))
        if self.supports_rtc_conditioning and not callable(getattr(policy, "predict_rtc", None)):
            raise ValueError("RTC capability requires predict_rtc")
        # This flag describes a declared model adapter, not validated task success.
        self.is_real_model = True

    @staticmethod
    def validate_config(config, *, embodiment="x5"):
        names = {
            "x5": ("arx_x5", "world_link6"),
            "piper": ("piper", "world_link6"),
            "franka": ("franka_panda", "world_panda_hand"),
        }
        if embodiment not in names:
            raise ValueError("unknown policy embodiment")
        robot, frame = names[embodiment]
        required = dict(
            robot=robot,
            action_frame=frame,
            action_representation="absolute_eef10",
            rotation_6d="first_two_columns_concatenated",
            gripper="minus1_closed_plus1_open",
            control_hz=30,
        )
        for key, expected in required.items():
            if config.get(key) != expected:
                raise ValueError(f"policy contract mismatch: {key} must be {expected!r}")
        for key in ("checkpoint", "checkpoint_sha256", "normalization", "training_data", "model_class"):
            if not isinstance(config.get(key), str) or not config[key].strip():
                raise ValueError(f"missing policy provenance: {key}")
        digest = config["checkpoint_sha256"]
        if len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ValueError("checkpoint_sha256 must be a full lowercase SHA-256")

    def predict(self, observation, request_tick, horizon):
        return np.asarray(self.policy.predict(observation, request_tick, horizon))

    def predict_rtc(self, observation, request_tick, horizon, **kwargs):
        return self.policy.predict_rtc(observation, request_tick, horizon, **kwargs)
