"""Deterministic clock regression tests; no simulator, GPU, or timing sleeps."""

from concurrent.futures import Future
from threading import Event
from types import SimpleNamespace

import numpy as np
import pytest

from dynamicbench.runtime import libero_scheduler as runtime
from dynamicbench.tests.test_libero_scheduler import Policy, observation


@pytest.fixture
def measured_worker(monkeypatch):
    clock = SimpleNamespace(now=100.0, due=100.25, waits=[])
    release = Event()
    monkeypatch.setattr(runtime, "time", SimpleNamespace(perf_counter=lambda: clock.now))

    class Worker(Policy):
        def predict(self, *args):
            assert release.wait(2), "test did not release model worker"
            return super().predict(*args)

    def paced_wait(futures, timeout):
        clock.waits.append(timeout)
        (future,) = futures
        if clock.now + timeout >= clock.due:
            clock.now = clock.due
            release.set()
            future.result(timeout=2)
            return {future}, set()
        clock.now += timeout
        return set(), {future}

    monkeypatch.setattr(runtime, "wait", paced_wait)
    yield clock, release, Worker()
    release.set()


@pytest.mark.parametrize("mode,prefix", [("async_queue", 0), ("rtc", 8)])
@pytest.mark.parametrize("simulation_speed", ["fast", "slow"])
def test_measured_250ms_releases_at_tick8_regardless_of_simulation_speed(
    measured_worker, mode, prefix, simulation_speed
):
    clock, release, policy = measured_worker
    scheduler = runtime.LiberoScheduler(policy, mode=mode, latency_source="measured_wallclock", threshold=0)
    try:
        assert scheduler.tick(0, observation())[0] is None
        if simulation_speed == "slow":
            # Model finishes at 250 ms; slow rendering returns to tick 1 at 3 s.
            clock.now = clock.due
            release.set()
            scheduler.pending["future"].result(timeout=2)
            clock.now = 103.0
        for tick in range(1, 8):
            assert scheduler.tick(tick, observation())[0] is None
        action, info = scheduler.tick(8, observation())
        request = scheduler.requests[0]
        assert request["inference_wall_seconds"] == 0.25
        assert request["latency_sim_seconds"] == 0.25
        assert request["available_tick"] == request["release_tick"] == 8
        assert request["effective_delay_steps"] == 8
        assert request["release_lateness_steps"] == 0
        assert request["dropped_prefix"] == prefix
        assert info["action_index"] == prefix
        assert action[0] == prefix / 100
        if simulation_speed == "slow":
            assert not clock.waits  # No second 250 ms delay after inference.
            assert request["clock_pacing_wall_seconds"] == 0
        else:
            assert clock.waits
            assert request["clock_pacing_wall_seconds"] == pytest.approx(0.25)
    finally:
        release.set()
        scheduler.close()


@pytest.mark.parametrize("mode", ["async_queue", "rtc"])
def test_existing_actions_keep_executing_during_measured_inference(measured_worker, mode):
    clock, release, policy = measured_worker
    scheduler = runtime.LiberoScheduler(policy, mode=mode, latency_source="measured_wallclock", threshold=16)
    old = Policy().predict({k: v for k, v in observation().items() if k != "oracle_debug"}, 10, 16)
    scheduler.queue = [
        dict(action=a, original=a.copy(), chunk_id=-1, action_index=i, request_tick=-16) for i, a in enumerate(old)
    ]
    try:
        for tick in range(8):
            action, info = scheduler.tick(tick, observation())
            np.testing.assert_array_equal(action, old[tick])
            assert not info["underflow"] and info["chunk_id"] == -1
        _, info = scheduler.tick(8, observation())
        assert info["action_index"] == 8
        assert scheduler.requests[0]["dropped_prefix"] == 8
        assert scheduler.requests[0]["overlap"] == (8 if mode == "async_queue" else 0)
    finally:
        release.set()
        scheduler.close()


@pytest.mark.parametrize("horizon,remaining", [(32, 12), (16, 0)])
def test_late_rtc_release_drops_actual_age_even_if_nominal_delay_is_smaller(horizon, remaining):
    scheduler = runtime.LiberoScheduler(
        Policy(), mode="rtc", latency_source="measured_wallclock", horizon=horizon, threshold=0
    )
    future = Future()
    clean_obs = {k: v for k, v in observation().items() if k != "oracle_debug"}
    future.set_result((Policy().predict(clean_obs, 0, horizon), 0.25))
    record = dict(request_tick=0, latency_sim_seconds=None, chunk_id=0, executed_at_submit=0)
    scheduler.pending = dict(record=record, future=future, result=None, timing={"policy_call_wall_seconds": 0.25})
    try:
        assert scheduler._collect(20)
        assert record["available_tick"] == 8 and record["release_tick"] == 20
        assert record["queue_delay_steps"] == 8  # Retain nominal latency for auditing.
        assert record["effective_delay_steps"] == 20 and record["release_lateness_steps"] == 12
        assert record["dropped_prefix"] == min(horizon, 20)
        assert len(scheduler.queue) == remaining
        if remaining:
            assert scheduler.queue[0]["action_index"] == 20
    finally:
        scheduler.close()


@pytest.mark.parametrize("mode,wait_time", [("sync_hold", 0.25), ("no_latency", 0.0)])
def test_sync_and_zero_delay_do_not_also_pace_async_ticks(monkeypatch, mode, wait_time):
    clock = SimpleNamespace(now=100.0)
    monkeypatch.setattr(runtime, "time", SimpleNamespace(perf_counter=lambda: clock.now))

    class Worker(Policy):
        def predict(self, *args):
            clock.now += 0.25
            return super().predict(*args)

    def forbidden_wait(*args, **kwargs):
        pytest.fail("sync/no_latency must not also use asynchronous clock pacing")

    monkeypatch.setattr(runtime, "wait", forbidden_wait)
    scheduler = runtime.LiberoScheduler(Worker(), mode=mode, latency_source="measured_wallclock")
    try:
        _, info = scheduler.tick(0, observation())
        assert info["sync_latency_seconds"] == wait_time
        assert info["action_index"] == 0
        assert scheduler.requests[0]["inference_wall_seconds"] == 0.25
        assert scheduler.requests[0]["effective_delay_steps"] == 0
        assert scheduler.requests[0]["clock_pacing_wall_seconds"] == 0
    finally:
        scheduler.close()


def test_output_materialization_is_included_in_response_ready_time(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(runtime, "time", SimpleNamespace(perf_counter=lambda: clock.now))

    class Materialized:
        def __init__(self, actions):
            self.actions = actions

        def __array__(self, dtype=None, copy=None):
            clock.now += 0.15
            return np.asarray(self.actions, dtype=dtype)

    class Worker(Policy):
        def predict(self, *args):
            clock.now += 0.1
            return Materialized(super().predict(*args))

    scheduler = runtime.LiberoScheduler(Worker(), mode="sync_hold", latency_source="measured_wallclock")
    try:
        _, info = scheduler.tick(0, observation())
        assert info["sync_latency_seconds"] == 0.25
        assert scheduler.requests[0]["policy_call_wall_seconds"] == 0.1
        assert scheduler.requests[0]["inference_wall_seconds"] == 0.25
    finally:
        scheduler.close()
