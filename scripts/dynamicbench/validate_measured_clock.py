#!/usr/bin/env python3
"""Conveyor integration audit using a timed mock, never a claimed real model.

The worker sleeps in wall time; measured_wallclock must derive its own delay.
The slow cases add wall time to physics execution without changing its dt.
"""

import json
import math
from pathlib import Path
import sys
from threading import Event
import time
import traceback

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


class TimedMock:
    is_real_model = False

    def __init__(self, seconds):
        from dynamicbench.runtime.mock_chunk_policy import MockChunkPolicy

        self.mock = MockChunkPolicy()
        self.seconds = seconds
        self.active = Event()
        self.provenance = dict(kind="timed_diagnostic_mock", worker_sleep_seconds=seconds)

    def predict(self, observation, request_tick, horizon):
        self.active.set()
        try:
            time.sleep(self.seconds)
            return self.mock.predict(observation, request_tick, horizon)
        finally:
            self.active.clear()


class InstrumentedBackend:
    def __init__(self, env, policy, extra_wall_seconds):
        self.env, self.policy = env, policy
        self.extra_wall_seconds = extra_wall_seconds
        self.steps = self.inflight_steps = 0

    def __getattr__(self, key):
        return getattr(self.env, key)

    def sim_step(self):
        if self.steps % 10 == 0 and self.extra_wall_seconds:
            time.sleep(self.extra_wall_seconds)
        # Count steps that actually start while the inference worker is active.
        self.inflight_steps += int(self.policy.active.is_set())
        self.env.sim_step()
        self.steps += 1


def audit(recorder, metrics, backend, requested_ticks):
    mode = metrics["latency_mode"]
    requests = [r for r in metrics["requests"] if r["status"] == "released"]
    actions = [t for t in recorder.trace if t["transition_kind"] == "action"]
    assert requests and len(actions) == requested_ticks, "incomplete diagnostic rollout"
    assert metrics["failure_reason"] == "timeout" and not metrics["real_model_tested"]
    expected_wait = sum(math.ceil(r["inference_wall_seconds"] * 300 - 1e-12) for r in requests)
    if mode != "sync_hold":
        expected_wait = 0
    assert metrics["sync_latency_physics_steps"] == expected_wait
    assert metrics["physics_steps"] == 10 * requested_ticks + expected_wait
    expected_move = np.asarray(backend.settings["belt"]["direction"]) * backend.speed * metrics["sim_time"]
    error = np.asarray(metrics["cube_displacement"]) - expected_move
    assert np.linalg.norm(error[:2]) < max(0.01, np.linalg.norm(expected_move[:2]) * 0.15), error
    assert abs(error[2]) < 0.01, error
    previous_q = None
    for row, q in zip(recorder.trace, recorder.joint_targets, strict=True):
        if row["transition_kind"] == "sync_wait" and previous_q is not None:
            np.testing.assert_array_equal(q, previous_q)
        previous_q = q
    old_actions_while_pending = 0
    for r in requests:
        assert r["inference_wall_seconds"] >= backend.policy.seconds
        assert r["policy_call_wall_seconds"] <= r["inference_wall_seconds"]
        prior_actions = sum(
            not t["underflow"] for t in actions if r["request_tick"] <= t["control_step"] < r["release_tick"]
        )
        old_actions_while_pending += prior_actions
        if mode in {"async_queue", "rtc"}:
            expected_delay = math.ceil(r["inference_wall_seconds"] * 30)
            assert r["queue_delay_steps"] == expected_delay
            assert r["available_tick"] == r["request_tick"] + expected_delay
            assert r["release_tick"] >= r["available_tick"]
            assert r["effective_delay_steps"] == r["release_tick"] - r["request_tick"]
            assert r["release_lateness_steps"] == r["release_tick"] - r["available_tick"]
            drop = r["effective_delay_steps"] if mode == "rtc" else prior_actions
            assert r["dropped_prefix"] == min(drop, metrics["action_horizon"])
            assert r["release_lateness_steps"] <= 1, "measured clock outran worker by more than one tick"
        else:
            assert r["release_tick"] == r["request_tick"] and r["dropped_prefix"] == 0
            if mode == "no_latency":
                assert r["latency_sim_seconds"] == 0
        for t in actions:
            if t["chunk_id"] == r["chunk_id"]:
                assert t["control_step"] >= r["release_tick"]
                assert t["action_index"] >= r["dropped_prefix"]
    if mode in {"async_queue", "rtc"}:
        assert backend.inflight_steps > 0, "no physical step overlapped inference"
        assert old_actions_while_pending > 0, "old action queue never ran during a pending request"
    return dict(
        passed=True,
        released_requests=len(requests),
        physics_steps_during_worker=backend.inflight_steps,
        old_queue_actions_during_pending=old_actions_while_pending,
        belt_displacement_error_m=error.tolist(),
        release_lateness_steps=[r["release_lateness_steps"] for r in requests],
    )


def main():
    from dynamicbench.cli import exception_details, finish, launch, load_settings, parser

    args, launcher, commit = launch(parser(evaluation=True))
    env = None
    try:
        if args.policy_factory or args.latency <= 0 or args.ticks < 64:
            raise ValueError("diagnostic requires no policy factory, positive --latency and at least 64 --ticks")
        from dynamicbench.data.dataset_reader import read_episode
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.episode import metadata, run_episode
        from dynamicbench.runtime.evaluation import run_evaluation

        env = ConveyorCubeTask(load_settings(args), launcher.app, args.seed, args.belt_speed)
        (args.output / "asset_audit.json").write_text(json.dumps(env.asset_audit, indent=2))
        cases = [(m, 0.0) for m in ("no_latency", "sync_hold", "async_queue", "rtc")]
        cases += [(m, 0.05) for m in ("async_queue", "rtc")]
        reports = []
        for i, (mode, throttle) in enumerate(cases):
            if i:
                env.reset(args.seed)
            name = mode + ("_slow" if throttle else "_normal")
            policy = TimedMock(args.latency)
            backend = InstrumentedBackend(env, policy, throttle)
            started = time.perf_counter()
            recorder, metrics = run_evaluation(
                backend,
                policy,
                ticks=args.ticks,
                mode=mode,
                horizon=args.horizon,
                threshold=args.prefetch_threshold,
                latency_source="measured_wallclock",
                latency=args.latency,
                seed=args.seed,
            )
            metrics.update(
                case=name,
                rollout_wall_seconds=time.perf_counter() - started,
                extra_wall_seconds_per_control_interval=throttle,
            )
            recorder.save(args.output, name, metadata(env, metrics, commit), video=not args.no_video)
            checks = audit(recorder, metrics, backend, args.ticks)
            _, _, schema = read_episode(args.output / name)
            reports.append(dict(**metrics, audit=checks, schema_audit=schema))
            (args.output / "metrics.json").write_text(json.dumps(reports, indent=2))
            print(json.dumps(dict(case=name, audit=checks, physics_steps=metrics["physics_steps"])), flush=True)
            del recorder
        # Regression of actual contact/lift behavior on the same code and scene.
        env.reset(args.seed)
        recorder, expert = run_episode(env, test="expert", ticks=600)
        recorder.save(args.output, "expert_regression", metadata(env, expert, commit), video=not args.no_video)
        _, _, expert_schema = read_episode(args.output / "expert_regression")
        (args.output / "expert_metrics.json").write_text(json.dumps(expert, indent=2))
        assert expert.get("passed"), "expert contact/lift regression failed"
        acceptance = dict(
            passed=True,
            source_commit=commit,
            real_model_tested=False,
            timing_scope="worker call plus CPU output materialization and validation; not end-to-end",
            cases=[dict(case=r["case"], audit=r["audit"]) for r in reports],
            expert_schema=expert_schema,
            expert_passed=expert["passed"],
        )
        (args.output / "acceptance.json").write_text(json.dumps(acceptance, indent=2))
        return finish(args, 0)
    except Exception as exc:
        (args.output / "error.json").write_text(json.dumps(exception_details(exc), indent=2))
        traceback.print_exc()
        return finish(args, 1)
    finally:
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    raise SystemExit(main())
