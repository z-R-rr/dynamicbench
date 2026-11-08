"""Evaluation-only timeline: explicit hold transitions and separate action ticks."""

import math

from dynamicbench.data.recorder import EpisodeRecorder
from dynamicbench.envs.fixed_rate_controller import ControlFailure, FixedRateController
from dynamicbench.runtime.libero_scheduler import REFERENCE_COMMIT, LiberoScheduler


def run_evaluation(
    env,
    policy,
    *,
    ticks=300,
    mode="async_queue",
    horizon=32,
    threshold=16,
    latency_source="fixed",
    latency=0.25,
    samples=None,
    seed=42,
):
    controller = FixedRateController(env, evaluation_timing=True)
    scheduler = LiberoScheduler(
        policy,
        mode=mode,
        horizon=horizon,
        threshold=threshold,
        latency_source=latency_source,
        latency=latency,
        samples=samples,
        seed=seed,
    )
    recorder = EpisodeRecorder()
    start = controller.observe()
    success, reason = False, None
    latency_steps = 0
    action_steps = 0

    def execute(action, info, *, hold, steps=10, kind="action"):
        fatal = None
        try:
            result = controller.step(action, hold=hold, physics_steps=steps)
        except ControlFailure as exc:
            result, fatal = exc.transition, str(exc)
        before, after = result["observation"], result["next_observation"]
        recorder.append(
            result,
            dict(
                **info,
                transition_kind=kind,
                sim_time=before["sim_time"],
                t_action_execute=before["sim_time"],
                physics_step=before["physics_step"],
                physics_steps=steps,
                end_sim_time=after["sim_time"],
                commanded_eef=result["action"],
                commanded_q=result["joint_target"],
                control_error=result["control_error"],
            ),
        )
        return after, fatal

    try:
        for k in range(ticks):
            obs = controller.observe()
            action, info = scheduler.tick(k, obs)
            wait = info["sync_latency_seconds"]
            # Fixed PhysX dt is retained. Error is in [0,1/300s), not 1/30s.
            remaining = math.ceil(wait * 300 - 1e-12) if wait else 0
            if wait:
                scheduler.requests[-1].update(
                    sync_physics_steps=remaining,
                    sync_applied_seconds=remaining / 300,
                    t_response_available=obs["sim_time"] + remaining / 300,
                )
            while remaining:
                count = min(10, remaining)
                _, fatal = execute(controller.previous_action, info, hold=True, steps=count, kind="sync_wait")
                latency_steps += count
                remaining -= count
                if fatal:
                    reason = fatal
                    break
            if reason:
                break
            after, fatal = execute(controller.previous_action if action is None else action, info, hold=action is None)
            action_steps += 1
            success, reason = env.outcome(after, k + 1)
            if fatal:
                success, reason = False, fatal
            if success or reason:
                break
    finally:
        scheduler.close()
    end = controller.observe()
    metrics = dict(
        schema_version="dynamicbench.eef10.eval.v2",
        test="policy_evaluation",
        reference_commit=REFERENCE_COMMIT,
        success=success,
        failure_reason=reason or (None if success else "timeout"),
        seed=env.seed_value,
        belt_speed=env.speed,
        control_actions=action_steps,
        recorded_transitions=len(recorder.actions),
        physics_steps=controller.clock.physics_step,
        sim_time=controller.clock.sim_time,
        sync_latency_physics_steps=latency_steps,
        underflow_steps=scheduler.underflows,
        executed_queue_actions=scheduler.executed,
        latency_mode=mode,
        latency_source=latency_source,
        latency_requested_seconds=latency,
        action_horizon=horizon,
        prefetch_threshold=threshold,
        requests=scheduler.requests,
        hold_semantics="finite_joint_drive_previous_target_no_state_writes",
        real_model_tested=bool(getattr(policy, "is_real_model", False)),
        policy=getattr(policy, "provenance", {"kind": "mock"}),
        object_displacement=(end["oracle_debug"]["cube_pose"][:3] - start["oracle_debug"]["cube_pose"][:3]).tolist(),
        cube_displacement=(end["oracle_debug"]["cube_pose"][:3] - start["oracle_debug"]["cube_pose"][:3]).tolist(),
    )
    return recorder, metrics
