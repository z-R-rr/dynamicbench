import pytest

from dynamicbench.envs.physics_clock import PhysicsClock, integral_ratio


def test_physical_clock_and_tolerance():
    c = PhysicsClock()
    times = []
    for k in range(30):
        assert c.control_tick == k
        times.append(c.sim_time)
        for _ in range(10):
            c.advance()
    assert c.physics_step == 300
    assert c.sim_time == 1
    assert c.control_tick == 30
    assert all(b > a for a, b in zip(times, times[1:]))
    assert integral_ratio(1 / 300, 30) == 10
    for dt, hz in [(1 / 299, 30), (1 / 300, 29.99), (0, 30), (-0.1, 30), (float("nan"), 30)]:
        with pytest.raises(ValueError):
            integral_ratio(dt, hz)
