"""LIBERO-Dynamic b66cb6b queue semantics; simulator-free, one inference worker.

Sync latency is returned to the executor, never charged to queue ticks. Async
availability uses control ticks. Underflow counts for RTC delay, but NOT for
the ordinary async executed-action prefix. Native Isaac drives remain finite.
"""

from concurrent.futures import ThreadPoolExecutor, wait
from copy import deepcopy
import math
from threading import Event
import time

import numpy as np

from dynamicbench.envs.poses import validate_eef10

REFERENCE_COMMIT = "b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4"


def blend_eef10(old, new):
    """Reference 0.3/0.7 overlap, projecting rotation columns back onto SO(3).

    LIBERO actions are Euclidean deltas. Absolute EEF10 rotation columns are
    constrained; projection is an explicit embodiment adaptation, not a claim
    that the two action spaces are identical.
    """
    out = 0.3 * old + 0.7 * new
    first = out[3:6]
    first = first / np.linalg.norm(first)
    second = out[6:9] - np.dot(first, out[6:9]) * first
    if np.linalg.norm(second) < 1e-8:
        raise ValueError("degenerate async rotation blend")
    out[3:6], out[6:9] = first, second / np.linalg.norm(second)
    return validate_eef10(out)


class LiberoScheduler:
    def __init__(
        self,
        policy,
        *,
        mode="async_queue",
        horizon=32,
        threshold=16,
        latency_source="fixed",
        latency=0.25,
        samples=None,
        seed=42,
        control_hz=30,
        worker_timeout=120,
    ):
        if mode not in {"no_latency", "sync_hold", "async_queue", "rtc"}:
            raise ValueError("invalid mode")
        if latency_source not in {"fixed", "empirical", "measured_wallclock"}:
            raise ValueError("invalid latency source")
        if not math.isfinite(latency) or latency < 0:
            raise ValueError("invalid latency")
        if not isinstance(horizon, int) or horizon < 1 or not 0 <= threshold <= horizon:
            raise ValueError("invalid horizon/threshold")
        self.samples = np.asarray(samples if samples is not None else [], dtype=float)
        if latency_source == "empirical" and (
            self.samples.ndim != 1
            or not len(self.samples)
            or not np.isfinite(self.samples).all()
            or (self.samples < 0).any()
        ):
            raise ValueError("empirical latency requires finite nonnegative samples")
        self.policy, self.mode = policy, mode
        self.horizon, self.threshold = horizon, threshold
        self.latency_source, self.latency = latency_source, latency
        self.hz, self.worker_timeout = control_hz, worker_timeout
        self.rng = np.random.default_rng(seed)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dynamicbench-policy")
        self.pending = None
        self.queue = []
        self.executed = self.underflows = 0
        self.previous_tick = -1
        self.requests = []
        self.closed = False

    def _submit(self, k, obs):
        # Explicit allowlist, independent copies, no scene/tensors/oracle state.
        snapshot = {key: deepcopy(obs[key]) for key in ("eef10", "qpos", "qvel", "head", "wrist") if key in obs}
        leftover = np.asarray([item["original"] for item in self.queue]).reshape(-1, 10).copy()
        latency = (
            float(self.rng.choice(self.samples))
            if self.latency_source == "empirical"
            else self.latency
            if self.latency_source == "fixed"
            else None
        )
        if self.mode == "no_latency":
            latency = 0.0
        rtc_used = self.mode == "rtc" and bool(getattr(self.policy, "supports_rtc_conditioning", False))
        if rtc_used and not callable(getattr(self.policy, "predict_rtc", None)):
            raise ValueError("policy advertises RTC without predict_rtc")
        request = dict(
            chunk_id=len(self.requests),
            request_tick=k,
            t_observe=obs["sim_time"],
            t_request=obs["sim_time"],
            executed_at_submit=self.executed,
            latency_sim_seconds=latency,
            rtc_conditioning_requested=self.mode == "rtc",
            rtc_conditioning_used=rtc_used,
            leftover_count=len(leftover),
            status="pending",
            clock_pacing_wall_seconds=0.0,
        )

        timing = {}
        clock_started = Event()

        def infer():
            started = time.perf_counter()
            timing["started_wall"] = started
            clock_started.set()
            if rtc_used:
                result = self.policy.predict_rtc(
                    snapshot,
                    k,
                    self.horizon,
                    prev_chunk_left_over=leftover,
                    inference_delay_steps=None if latency is None else math.ceil(latency * self.hz),
                )
            else:
                result = self.policy.predict(snapshot, k, self.horizon)
            timing["policy_call_wall_seconds"] = time.perf_counter() - started
            actions = np.asarray(result, dtype=float)
            if actions.shape != (self.horizon, 10):
                raise ValueError("policy must return exactly [horizon,10] absolute EEF10")
            actions = np.asarray([validate_eef10(row) for row in actions])
            # The response is usable only after CPU materialization/validation.
            # Adapters must return completed outputs, not asynchronous GPU handles.
            return actions, time.perf_counter() - started

        self.requests.append(request)
        self.pending = dict(
            record=request,
            future=self.executor.submit(infer),
            result=None,
            timing=timing,
            clock_started=clock_started,
        )

    def _pace_measured_request(self, k, pending):
        """Do not outrun an unfinished measured inference in simulation time.

        A control interval may cross the completion instant by less than one
        tick. Slow simulation catches up after the future completes without an
        additional wall-clock sleep. Fixed/empirical and sync waits bypass this.
        """
        future, record = pending["future"], pending["record"]
        if future.done():
            return
        started_wait = time.perf_counter()
        if not pending["clock_started"].wait(self.worker_timeout):
            raise TimeoutError("policy worker did not start its measured clock")
        deadline = pending["timing"]["started_wall"] + (k - record["request_tick"]) / self.hz
        remaining = deadline - time.perf_counter()
        if remaining > 0:
            # Wake either when this control boundary is due or inference ends.
            wait([future], timeout=remaining)
        record["clock_pacing_wall_seconds"] += max(0.0, time.perf_counter() - started_wait)

    def _collect(self, k, *, force=False):
        p = self.pending
        if p is None:
            return False
        r = p["record"]
        latency = r["latency_sim_seconds"]
        if latency is None and not force:
            self._pace_measured_request(k, p)
        available = None if latency is None else r["request_tick"] + math.ceil(latency * self.hz)
        should_wait = force or (available is not None and k >= available)
        if p["result"] is None:
            if not p["future"].done() and not should_wait:
                return False
            started = time.perf_counter()
            p["result"], wall = p["future"].result(timeout=self.worker_timeout)
            r["worker_wait_wall_seconds"] = time.perf_counter() - started
            r["inference_wall_seconds"] = wall
            r["policy_call_wall_seconds"] = p["timing"]["policy_call_wall_seconds"]
            if latency is None:
                latency = wall
                r["latency_sim_seconds"] = latency
                available = r["request_tick"] + math.ceil(latency * self.hz)
        if not force and k < available:
            return False
        r["available_tick"] = k if self.mode in {"sync_hold", "no_latency"} else available
        r["release_tick"] = k
        r["effective_delay_steps"] = 0 if self.mode in {"sync_hold", "no_latency"} else k - r["request_tick"]
        r["release_lateness_steps"] = 0 if self.mode in {"sync_hold", "no_latency"} else max(0, k - available)
        r["queue_delay_steps"] = 0 if self.mode in {"sync_hold", "no_latency"} else available - r["request_tick"]
        self._merge(p["result"], r)
        r["status"] = "released"
        self.pending = None
        return True

    def _merge(self, actions, r):
        drop = (
            self.executed - r["executed_at_submit"]
            if self.mode == "async_queue"
            else r["effective_delay_steps"]
            if self.mode == "rtc"
            else 0
        )
        drop = min(drop, len(actions))
        incoming = [
            dict(
                action=a.copy(),
                original=a.copy(),
                chunk_id=r["chunk_id"],
                action_index=i,
                request_tick=r["request_tick"],
            )
            for i, a in enumerate(actions)
            if i >= drop
        ]
        overlap = min(len(self.queue), len(incoming)) if self.mode == "async_queue" else 0
        for i in range(overlap):
            incoming[i]["action"] = blend_eef10(self.queue[i]["action"], incoming[i]["action"])
            incoming[i]["blend_old_chunk_id"] = self.queue[i]["chunk_id"]
        self.queue = incoming
        r.update(dropped_prefix=drop, overlap=overlap, enqueued=len(incoming))

    def tick(self, k, observation):
        if self.closed or k != self.previous_tick + 1:
            raise RuntimeError("scheduler closed or nonsequential tick")
        self.previous_tick = k
        sync_latency = 0.0
        if self.mode not in {"sync_hold", "no_latency"}:
            self._collect(k)
        threshold = 0 if self.mode in {"sync_hold", "no_latency"} else self.threshold
        if self.pending is None and len(self.queue) <= threshold:
            self._submit(k, observation)
            if self.mode in {"sync_hold", "no_latency"}:
                self._collect(k, force=True)
                sync_latency = self.requests[-1]["latency_sim_seconds"]
            else:
                self._collect(k)
        before = len(self.queue)
        item = self.queue.pop(0) if self.queue else None
        if item is None:
            self.underflows += 1
        else:
            self.executed += 1
        info = dict(
            control_step=k,
            queue_before=before,
            queue_after=len(self.queue),
            underflow=item is None,
            sync_latency_seconds=sync_latency,
            action_source="finite_joint_hold" if item is None else "queue",
            chunk_id=None if item is None else item["chunk_id"],
            action_index=None if item is None else item["action_index"],
        )
        return None if item is None else item["action"].copy(), info

    def close(self):
        self.closed = True
        if self.pending is not None:
            self.pending["record"]["status"] = "not_applied_episode_ended"
        self.executor.shutdown(wait=True, cancel_futures=True)
