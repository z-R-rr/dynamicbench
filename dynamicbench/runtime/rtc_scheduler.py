"""Fixed simulated delay; this is NOT a benchmark of GPU inference overlap.

Actions in RTC chunks are indexed at request_tick+i. Returning at tick a drops
indices before a exactly once. Physics is advanced only by FixedRateController.
"""

from copy import deepcopy
from dataclasses import dataclass
import math
import time

import numpy as np

from dynamicbench.envs.poses import validate_eef10


@dataclass
class QueuedAction:
    action: np.ndarray
    chunk_id: int
    action_index: int
    request_tick: int
    available_tick: int
    inference_wall_ms: float


class RTCScheduler:
    def __init__(self, mode="rtc_async", latency=0.25, horizon=32, replan_stride=16, commit_ticks=None):
        if mode not in ("no_latency", "sync_hold", "rtc_async"):
            raise ValueError("unknown scheduling mode")
        if not math.isfinite(latency) or latency < 0:
            raise ValueError("invalid latency")
        if horizon <= 0 or replan_stride <= 0 or replan_stride > horizon:
            raise ValueError("invalid horizon/stride")
        self.mode, self.latency = mode, latency
        # Tiny tolerance only removes machine epsilon at exact integer delays.
        self.delay = 0 if mode == "no_latency" else math.ceil(latency * 30 - 1e-12)
        self.horizon, self.stride = horizon, replan_stride
        self.commit_ticks = self.delay if commit_ticks is None else int(commit_ticks)
        if self.commit_ticks < 0:
            raise ValueError("invalid commitment")
        self.queue = {}
        self.pending = None
        self.last_action = None
        self.next_id = 0
        self.previous_tick = -1
        self.next_request = 0
        self.requests = []

    def _release(self, k):
        if self.pending is None or k < self.pending["available"]:
            return
        p = self.pending
        if self.mode == "sync_hold":
            start, skip = k, 0
        else:
            start, skip = max(k, p["commit_until"]), max(k, p["commit_until"]) - p["request"]
        self.queue = {tick: a for tick, a in self.queue.items() if k <= tick < start}
        for i in range(skip, len(p["actions"])):
            tick = start + i - skip
            self.queue[tick] = QueuedAction(p["actions"][i].copy(), p["id"], i, p["request"], p["available"], p["wall"])
        self.pending = None

    def tick(self, k, observation, policy):
        if k != self.previous_tick + 1:
            raise RuntimeError("scheduler tick skipped, repeated, or rolled back")
        self.previous_tick = k
        if self.last_action is None:
            self.last_action = validate_eef10(observation["eef10"])
        self._release(k)
        request = self.pending is None and (not self.queue if self.mode == "sync_hold" else k >= self.next_request)
        if request:
            # Only learned-policy fields, NEVER oracle_debug or future physics.
            snapshot = {
                key: deepcopy(observation[key]) for key in ("eef10", "qpos", "qvel", "head") if key in observation
            }
            t0 = time.perf_counter()
            actions = np.asarray(policy.predict(snapshot, k, self.horizon), dtype=float)
            wall = (time.perf_counter() - t0) * 1000
            if actions.shape != (self.horizon, 10):
                raise ValueError("policy must return [horizon,10] absolute EEF commands")
            actions = np.asarray([validate_eef10(a) for a in actions])
            self.pending = dict(
                actions=actions,
                request=k,
                available=k + self.delay,
                id=self.next_id,
                commit_until=k + self.commit_ticks,
                wall=wall,
            )
            self.requests.append(
                dict(
                    chunk_id=self.next_id,
                    t_observe=k / 30,
                    t_request=k / 30,
                    chunk_request_tick=k,
                    chunk_available_tick=k + self.delay,
                    t_response_available=(k + self.delay) / 30,
                    inference_wall_ms=wall,
                    latency_sim_ms=self.delay / 30 * 1000,
                    latency_requested_ms=self.latency * 1000,
                )
            )
            self.next_id += 1
            self.next_request = k + self.stride
            self._release(k)
        item = self.queue.pop(k, None)
        if item is None:
            action, hold = self.last_action.copy(), True
            info = dict(
                chunk_id=None,
                action_index=None,
                chunk_request_tick=None,
                chunk_available_tick=None,
                inference_wall_ms=None,
                latency_sim_ms=None,
            )
        else:
            action, hold = item.action.copy(), False
            self.last_action = action.copy()
            info = dict(
                chunk_id=item.chunk_id,
                action_index=item.action_index,
                chunk_request_tick=item.request_tick,
                chunk_available_tick=item.available_tick,
                inference_wall_ms=item.inference_wall_ms,
                latency_sim_ms=(item.available_tick - item.request_tick) / 30 * 1000,
            )
        return action, dict(
            **info,
            queue_length=len(self.queue),
            hold=hold,
            action_source="safe_hold" if hold else "chunk",
            t_action_execute=k / 30,
        )
