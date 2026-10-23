import numpy as np
import pytest
from dynamicbench.envs.fixed_rate_controller import FixedRateController
from dynamicbench.envs.poses import encode_eef10

class Backend:
    generation=0
    physics_step=0
    fail=False
    def __init__(self):
        self.commands=[]
    def actual_joint_target(self):
        return np.array([.1,.2])
    def observe(self):
        return dict(eef10=encode_eef10([.1,.2,.3],np.eye(3),1),qpos=np.array([.1,.2]),qvel=np.zeros(2))
    def solve_target(self,a):
        return (None,'ik_failure') if self.fail else (np.array([.5,.6]),None)
    def set_joint_target(self,q):
        self.commands.append(q.copy())
    def before_physics_step(self):
        pass
    def sim_step(self):
        self.physics_step+=1


def test_zoh_observe_action_next_alignment():
    b=Backend()
    c=FixedRateController(b)
    r=c.step(b.observe()['eef10'])
    assert len(b.commands)==10
    np.testing.assert_equal(b.commands,np.tile([.5,.6],(10,1)))
    assert r['observation']['sim_time']==0
    assert r['next_observation']['sim_time']==1/30
    # Actual feedback remains distinct from command in this instrumented backend.
    assert not np.array_equal(r['joint_target'],r['next_observation']['qpos'])


def test_failed_ik_retains_legal_command_and_reset_is_detected():
    b=Backend(); c=FixedRateController(b,max_ik_failures=2)
    c.step(b.observe()['eef10'])
    b.fail=True
    r=c.step(b.observe()['eef10'])
    assert r['control_error']=='ik_failure'
    np.testing.assert_equal(b.commands[-1],[.5,.6])
    with pytest.raises(RuntimeError,match='consecutive_ik'):
        c.step(b.observe()['eef10'])
    b.generation+=1
    with pytest.raises(RuntimeError,match='reset'):
        c.observe()


def test_external_physics_step_invalidates_clock():
    b=Backend(); c=FixedRateController(b)
    b.physics_step+=1
    with pytest.raises(RuntimeError,match='owner'):
        c.observe()


def test_partial_step_exception_poisons_controller():
    b=Backend(); c=FixedRateController(b)
    def crash():
        b.physics_step+=1
        raise RuntimeError('physx failure')
    b.sim_step=crash
    with pytest.raises(RuntimeError,match='physx'):
        c.step(b.observe()['eef10'])
    with pytest.raises(RuntimeError,match='invalid_after'):
        c.observe()
