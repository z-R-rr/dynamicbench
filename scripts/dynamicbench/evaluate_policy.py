#!/usr/bin/env python3
"""Separate v2 evaluation entrypoint; legacy expert collection stays uniform."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    import json
    import traceback

    from dynamicbench.cli import exception_details, finish, launch, load_settings, parser

    p = parser(evaluation=True)
    args, launcher, commit = launch(p)
    env = None
    try:
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.episode import metadata
        from dynamicbench.runtime.evaluation import run_evaluation
        from dynamicbench.runtime.mock_chunk_policy import MockChunkPolicy

        if args.policy_factory:
            if args.validate_scheduling:
                raise ValueError("scheduling validation uses a deterministic mock, not a real model")
            import importlib

            if not args.policy_config:
                raise ValueError("real policy factory requires a provenance/config JSON")
            from dynamicbench.runtime.policy_contract import CheckedPolicy

            config = json.loads(args.policy_config.read_text())
            module, name = args.policy_factory.split(":")
            factory = getattr(importlib.import_module(module), name)
            policy = CheckedPolicy(factory(config), config)
        else:
            policy = MockChunkPolicy()
        samples = json.loads(args.latency_samples.read_text()) if args.latency_samples else None
        env = ConveyorCubeTask(load_settings(args), launcher.app, args.seed, args.belt_speed)
        (args.output / "asset_audit.json").write_text(json.dumps(env.asset_audit, indent=2))
        modes = ["no_latency", "sync_hold", "async_queue", "rtc"] if args.validate_scheduling else [args.execution_mode]
        reports = []
        for i, mode in enumerate(modes):
            if i:
                env.reset(args.seed)
                policy = MockChunkPolicy()
            recorder, metrics = run_evaluation(
                env,
                policy,
                ticks=args.ticks,
                mode=mode,
                horizon=args.horizon,
                threshold=args.prefetch_threshold,
                latency_source=args.latency_source,
                latency=args.latency,
                samples=samples,
                seed=args.seed,
            )
            recorder.save(
                args.output, f"{mode}_seed{args.seed}", metadata(env, metrics, commit), video=not args.no_video
            )
            reports.append(metrics)
            (args.output / "metrics.json").write_text(json.dumps(reports, indent=2))
            if args.validate_scheduling:
                expected_steps = metrics["control_actions"] * 10 + metrics["sync_latency_physics_steps"]
                if expected_steps != metrics["physics_steps"]:
                    raise RuntimeError("physical timeline mismatch")
                expected_x = args.belt_speed * metrics["sim_time"]
                if abs(metrics["cube_displacement"][0] - expected_x) > max(0.01, abs(expected_x) * 0.15):
                    raise RuntimeError("belt did not advance through the latency timeline")
                if metrics["failure_reason"] != "timeout":
                    raise RuntimeError("unexpected mock rollout termination")
        if args.validate_scheduling:
            (args.output / "acceptance.json").write_text(
                json.dumps(dict(passed=True, modes=modes, real_model_tested=False))
            )
        # Task failure is a valid evaluation result, unlike runtime/contract errors.
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
