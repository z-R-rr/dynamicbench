"""One owner, observation-before-action, ten native sim_step calls, ZOH."""
import numpy as np
from .physics_clock import PhysicsClock
from .poses import validate_eef10


class FixedRateController:
    def __init__(self, backend, max_ik_failures=5):
        self.backend = backend
        self.clock = PhysicsClock()
        self.generation = backend.generation
        self.origin_step = backend.physics_step
        self.previous_q = np.asarray(backend.actual_joint_target()).copy()
        self.previous_action = validate_eef10(backend.observe()['eef10'])
        self.failures = 0
        self.max_ik_failures = max_ik_failures
        self.poisoned = False

    def observe(self):
        self._check()
        value = self.backend.observe()
        for key in ('eef10', 'qpos', 'qvel'):
            if not np.isfinite(value[key]).all():
                raise RuntimeError('numerical_instability')
        return {**value, **self.clock.stamp()}

    def _check(self):
        if self.poisoned:
            raise RuntimeError('controller_invalid_after_partial_step')
        if self.generation != self.backend.generation:
            raise RuntimeError('controller_reset_requires_reconstruction')
        if self.backend.physics_step != self.origin_step + self.clock.physics_step:
            raise RuntimeError('physics_clock_owner_violation')

    def step(self, action, *, hold=False):
        self._check()
        action = validate_eef10(action)
        before = self.observe()
        error = None
        if hold:
            action, q = self.previous_action.copy(), self.previous_q.copy()
        else:
            q, error = self.backend.solve_target(action)
            if q is None:
                action, q = self.previous_action.copy(), self.previous_q.copy()
                self.failures += 1
            else:
                q = np.asarray(q).copy()
                if q.shape != self.previous_q.shape or not np.isfinite(q).all():
                    raise RuntimeError('invalid_joint_target')
                self.failures = 0
        self.previous_action, self.previous_q = action.copy(), q.copy()
        try:
            for _ in range(10):
                self.backend.set_joint_target(q)
                self.backend.before_physics_step()
                self.backend.sim_step()
                self.clock.advance()
                self._check()
        except Exception:
            self.poisoned = True
            raise
        after = self.observe()
        if self.failures >= self.max_ik_failures:
            raise RuntimeError(f'consecutive_ik_failure:{error}')
        return dict(observation=before, action=action, joint_target=q,
                    next_observation=after, control_error=error)
