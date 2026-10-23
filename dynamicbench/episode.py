"""Shared evaluation and expert collection loop over the identical executor."""

import json

from dynamicbench.data.recorder import EpisodeRecorder
from dynamicbench.envs.fixed_rate_controller import ControlFailure, FixedRateController
from dynamicbench.experts.conveyor_cube_expert import ConveyorCubeExpert
from dynamicbench.runtime.mock_chunk_policy import MockChunkPolicy
from dynamicbench.runtime.rtc_scheduler import RTCScheduler


def run_episode(env, *, test="expert", ticks=600, mode="no_latency", latency=0.25):
    if test == "expert" and mode != "no_latency":
        raise ValueError("expert collection is no_latency only; use --test schedule for mock latency evaluation")
    controller = FixedRateController(env)
    expert = ConveyorCubeExpert(env.settings, env.belt_top)
    scheduler = RTCScheduler(mode, latency)
    policy = MockChunkPolicy()
    recorder = EpisodeRecorder()
    start = controller.observe()
    success, reason = False, None
    for k in range(ticks):
        obs = controller.observe()
        if test == "belt":
            action, hold, info = start["eef10"], True, {}
        elif test == "expert":
            action, hold, info = expert.action(obs), False, {}
        else:
            action, info = scheduler.tick(k, obs, policy)
            hold = info["hold"]
        fatal = None
        try:
            result = controller.step(action, hold=hold)
        except ControlFailure as exc:
            result, fatal = exc.transition, str(exc)
        trace = dict(
            sim_time=obs["sim_time"],
            physics_step=obs["physics_step"],
            control_tick=k,
            phase=expert.phase,
            robot_actual_pose=obs["eef10"],
            commanded_eef=result["action"],
            commanded_q=result["joint_target"],
            actual_q=obs["qpos"],
            cube_pose=obs["oracle_debug"]["cube_pose"],
            finger_contact_forces=obs["oracle_debug"]["finger_contact_forces"],
            belt_speed=env.speed,
            control_error=result["control_error"],
            **info,
        )
        recorder.append(result, trace)
        if k % 30 == 0:
            print(json.dumps(recorder.trace[-1]), flush=True)
        success, reason = env.outcome(result["next_observation"], k + 1)
        if fatal:
            success, reason = False, fatal
            break
        if test == "expert" and (success or reason or expert.failure):
            reason = reason or expert.failure
            break
    end = recorder.observations[-1]
    displacement = end["oracle_debug"]["cube_pose"][:3] - start["oracle_debug"]["cube_pose"][:3]
    metrics = dict(
        test=test,
        success=success,
        failure_reason=reason or (None if success else "diagnostic_only" if test != "expert" else "timeout"),
        seed=env.seed_value,
        belt_speed=env.speed,
        control_ticks=len(recorder.actions),
        physics_steps=controller.clock.physics_step,
        sim_time=controller.clock.sim_time,
        cube_displacement=displacement.tolist(),
        max_cube_z=env.max_cube_z,
        mean_cube_speed=float(displacement[0] / controller.clock.sim_time),
        latency_mode=mode,
        latency_requested_seconds=latency,
        latency_quantized_seconds=scheduler.delay / 30,
        real_model_tested=False,
        requests=scheduler.requests,
    )
    if test == "belt":
        expected = env.speed * controller.clock.sim_time
        metrics.update(
            expected_displacement_x=expected,
            passed=bool(
                abs(displacement[0] - expected) < max(0.01, abs(expected) * 0.15)
                and abs(displacement[1]) < 0.01
                and abs(displacement[2]) < 0.01
            ),
        )
    return recorder, metrics


def metadata(env, metrics, commit):
    cfg = env.settings["expert"]
    return dict(
        **metrics,
        task="pick_cube_from_conveyor",
        source_commit=commit,
        settings=env.settings,
        belt_top=env.belt_top,
        cube_size=env.settings["cube"]["size"],
        success_height=cfg["success_height"],
        success_hold_ticks=cfg["success_hold_ticks"],
        gripper_convention="-1 closed +1 open; measured joint7/8",
        frame="right_handed_world_link6",
        rotation_6d="first_two_columns_concatenated",
    )
