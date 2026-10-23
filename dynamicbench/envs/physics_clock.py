"""Integer physics steps are the only source of episode time."""

from dataclasses import dataclass
import math


def integral_ratio(dt: float, hz: float, expected: int = 10) -> int:
    if not math.isfinite(dt * hz) or dt <= 0 or hz <= 0:
        raise ValueError("invalid frequency")
    ratio = 1.0 / (dt * hz)
    if not math.isclose(ratio, expected, rel_tol=0, abs_tol=1e-10):
        raise ValueError(f"expected {expected} physics steps, got {ratio}")
    return expected


@dataclass
class PhysicsClock:
    physics_hz: int = 300
    control_hz: int = 30
    physics_step: int = 0

    def __post_init__(self):
        integral_ratio(1 / self.physics_hz, self.control_hz)

    @property
    def sim_time(self):
        return self.physics_step / self.physics_hz

    @property
    def control_tick(self):
        if self.physics_step % 10:
            raise RuntimeError("not at a control boundary")
        return self.physics_step // 10

    def advance(self):
        self.physics_step += 1

    def stamp(self):
        return dict(sim_time=self.sim_time, physics_step=self.physics_step, control_tick=self.control_tick)
