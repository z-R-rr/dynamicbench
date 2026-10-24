#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    from collections import Counter
    import fcntl
    import json
    import traceback

    from dynamicbench.cli import launch, load_settings, parser

    p = parser(collect=True)
    args, launcher, commit = launch(p)
    if args.num_success < 1 or args.max_attempts < 1:
        p.error("collection counts must be positive")
    env = None
    lock = (args.output / ".collector.lock").open("a")
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    attempts = []
    successes = 0
    try:
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.episode import metadata, run_episode

        env = ConveyorCubeTask(load_settings(args), launcher.app, args.seed, args.belt_speed)
        (args.output / "asset_audit.json").write_text(json.dumps(env.asset_audit, indent=2))
        for i in range(args.max_attempts):
            if i:
                env.reset(args.seed + i)
            recorder, metrics = run_episode(env, ticks=args.ticks)
            if metrics["success"] or args.save_failures:
                entry = recorder.save(
                    args.output, f"episode_seed{args.seed + i}", metadata(env, metrics, commit), video=not args.no_video
                )
                metrics["episode"] = entry["episode"]
            attempts.append(metrics)
            successes += int(metrics["success"])
            failures = Counter(x["failure_reason"] for x in attempts if not x["success"])
            summary = dict(
                source_commit=commit,
                successes=successes,
                attempts=len(attempts),
                success_rate=successes / len(attempts),
                total_control_ticks=sum(x["control_ticks"] for x in attempts),
                mean_success_duration=(
                    sum(x["sim_time"] for x in attempts if x["success"]) / successes if successes else None
                ),
                failure_counts=dict(failures),
                episodes=attempts,
            )
            temp = args.output / ".collection_metrics.tmp"
            temp.write_text(json.dumps(summary, indent=2))
            temp.replace(args.output / "collection_metrics.json")
            print(json.dumps(summary), flush=True)
            if successes >= args.num_success:
                return 0
        return 2
    except Exception as exc:
        (args.output / "collection_error.json").write_text(
            json.dumps(dict(error=repr(exc), traceback=traceback.format_exc(), completed_attempts=attempts), indent=2)
        )
        traceback.print_exc()
        return 1
    finally:
        lock.close()
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    raise SystemExit(main())
