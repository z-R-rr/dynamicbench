import numpy as np
import pytest

from dynamicbench.envs.poses import encode_eef10
from dynamicbench.runtime.rtc_scheduler import RTCScheduler


class Policy:
    def predict(self, obs, k, h):
        assert "oracle_debug" not in obs
        a = np.tile(obs["eef10"], (h, 1))
        a[:, 0] = k + np.arange(h)
        return a


def obs():
    return dict(eef10=encode_eef10([0.1, 0.2, 0.3], np.eye(3), 1), oracle_debug="secret")


def test_250ms_8ticks_old_prefix_not_rewound():
    s = RTCScheduler("rtc_async", 0.25)
    records = [s.tick(k, obs(), Policy()) for k in range(40)]
    assert all(r[1]["hold"] for r in records[:8])
    assert records[8][0][0] == 8
    assert records[8][1]["chunk_available_tick"] == 8
    # New chunk requested at 16 is gated until 24. Old chunk continues 16..23.
    assert all(records[k][1]["chunk_id"] == 0 for k in range(8, 24))
    assert records[24][1]["chunk_id"] == 1
    assert records[24][1]["action_index"] == 8
    assert [records[k][0][0] for k in range(8, 40)] == list(range(8, 40))


def test_committed_future_prefix_is_preserved():
    s = RTCScheduler("rtc_async", 0, horizon=32, replan_stride=16, commit_ticks=5)
    rows = [s.tick(k, obs(), Policy()) for k in range(23)]
    assert all(rows[k][1]["chunk_id"] == 0 for k in range(16, 21))
    assert rows[21][1]["chunk_id"] == 1
    assert rows[21][0][0] == 21


def test_sync_hold_uses_full_chunk_after_quantized_hold():
    s = RTCScheduler("sync_hold", 0.25)
    rows = [s.tick(k, obs(), Policy()) for k in range(49)]
    assert rows[8][0][0] == 0
    assert rows[39][0][0] == 31
    assert all(rows[k][1]["hold"] for k in range(40, 48))
    assert rows[48][0][0] == 40


def test_no_latency_is_immediate_and_time_must_be_sequential():
    s = RTCScheduler("no_latency", 0.5)
    assert not s.tick(0, obs(), Policy())[1]["hold"]
    with pytest.raises(RuntimeError):
        s.tick(0, obs(), Policy())


@pytest.mark.parametrize("delay,ticks", [(0, 0), (0.1, 3), (0.25, 8), (0.5, 15)])
def test_delay_quantization(delay, ticks):
    assert RTCScheduler(latency=delay).delay == ticks
