from threading import Event

import numpy as np
import pytest

from dynamicbench.envs.fixed_rate_controller import FixedRateController
from dynamicbench.envs.poses import encode_eef10
from dynamicbench.runtime.libero_scheduler import LiberoScheduler
from dynamicbench.tests.test_zoh import Backend


def observation():
    return dict(
        eef10=encode_eef10([0, 0, 1], np.eye(3), 1),
        sim_time=0,
        head=np.ones((2, 3, 3), dtype=np.uint8),
        wrist=np.ones((2, 3, 3), dtype=np.uint8),
        oracle_debug={"forbidden": True},
    )


class Policy:
    def predict(self, obs, tick, horizon):
        assert "oracle_debug" not in obs and "wrist" in obs
        a = np.tile(obs["eef10"], (horizon, 1))
        a[:, 0] = tick + np.arange(horizon) / 100
        return a


@pytest.mark.parametrize("mode,prefix", [("async_queue", 0), ("rtc", 8)])
def test_first_request_underflow_is_not_async_action_expiry(mode, prefix):
    s = LiberoScheduler(Policy(), mode=mode, latency=0.25, horizon=16, threshold=0)
    try:
        for k in range(8):
            action, info = s.tick(k, observation())
            assert action is None and info["underflow"]
        action, info = s.tick(8, observation())
        assert info["action_index"] == prefix
        assert action[0] == prefix / 100
        assert s.requests[0]["dropped_prefix"] == prefix
        assert s.requests[0]["queue_delay_steps"] == 8
    finally:
        s.close()


def test_sync_wait_is_physics_time_not_double_queue_delay():
    s = LiberoScheduler(Policy(), mode="sync_hold", latency=0.25, horizon=4, threshold=0)
    try:
        action, info = s.tick(0, observation())
        assert info["sync_latency_seconds"] == 0.25 and info["action_index"] == 0
        assert s.requests[0]["queue_delay_steps"] == 0
        b = Backend()
        c = FixedRateController(b, evaluation_timing=True)
        for steps in [10] * 7 + [5]:
            c.step(c.previous_action, hold=True, physics_steps=steps)
        assert c.observe()["sim_time"] == 0.25
        c.step(action)
        assert c.clock.physics_step == 85
        assert c.observe()["control_tick"] == 9
        np.testing.assert_array_equal(b.commands[:75], np.tile([0.1, 0.2], (75, 1)))
    finally:
        s.close()


def test_async_weighted_overlap_and_executed_prefix():
    s = LiberoScheduler(Policy(), mode="async_queue", latency=0, horizon=4, threshold=3)
    try:
        first, _ = s.tick(0, observation())
        assert first[0] == 0
        second, _ = s.tick(1, observation())
        assert second[0] == pytest.approx(0.3 * 0.01 + 0.7 * 1)
        assert s.requests[1]["overlap"] == 3
        assert s.requests[1]["dropped_prefix"] == 0
        assert np.linalg.norm(second[3:6]) == 1
    finally:
        s.close()


def test_rtc_conditioning_gets_unconsumed_original_actions_and_delay():
    class RTC(Policy):
        supports_rtc_conditioning = True

        def predict_rtc(self, obs, tick, horizon, *, prev_chunk_left_over, inference_delay_steps):
            self.leftover = prev_chunk_left_over.copy()
            assert inference_delay_steps == 0
            return self.predict(obs, tick, horizon)

    p = RTC()
    s = LiberoScheduler(p, mode="rtc", latency=0, horizon=4, threshold=3)
    try:
        s.tick(0, observation())
        action, _ = s.tick(1, observation())
        np.testing.assert_allclose(p.leftover[:, 0], [0.01, 0.02, 0.03])
        assert action[0] == 1  # No async blend in RTC.
        assert s.requests[-1]["rtc_conditioning_used"]
    finally:
        s.close()


def test_worker_receives_copy_and_does_not_block_early_ticks():
    entered, release = Event(), Event()

    class Delayed(Policy):
        def predict(self, obs, tick, horizon):
            entered.set()
            assert release.wait(5)
            assert obs["head"].sum() == 18
            return super().predict(obs, tick, horizon)

    s = LiberoScheduler(Delayed(), latency=0.25, horizon=16, threshold=0)
    try:
        obs = observation()
        assert s.tick(0, obs)[0] is None
        assert entered.wait(5)
        obs["head"][:] = 0
        assert s.tick(1, obs)[0] is None
        release.set()
    finally:
        release.set()
        s.close()


def test_expired_rtc_chunk_and_sequential_guard():
    s = LiberoScheduler(Policy(), mode="rtc", latency=0.25, horizon=4, threshold=0)
    try:
        for k in range(9):
            assert s.tick(k, observation())[0] is None
        assert s.requests[0]["dropped_prefix"] == 4
        with pytest.raises(RuntimeError, match="nonsequential"):
            s.tick(8, observation())
    finally:
        s.close()
