#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    import json
    import traceback

    from dynamicbench.cli import launch, load_settings, parser

    args, launcher, commit = launch(parser())
    env = None
    try:
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.episode import metadata, run_episode

        env = ConveyorCubeTask(load_settings(args), launcher.app, args.seed, args.belt_speed)
        (args.output / "asset_audit.json").write_text(json.dumps(env.asset_audit, indent=2))
        recorder, metrics = run_episode(
            env, test=args.test, ticks=args.ticks, mode=args.latency_mode, latency=args.latency
        )
        entry = recorder.save(
            args.output, f"episode_seed{args.seed}", metadata(env, metrics, commit), video=not args.no_video
        )
        (args.output / "metrics.json").write_text(json.dumps(metrics, indent=2))
        print(json.dumps(dict(metrics=metrics, episode=entry)), flush=True)
        return 0 if metrics.get("passed", metrics["success"]) or args.test == "schedule" else 2
    except Exception as exc:
        (args.output / "error.json").write_text(
            json.dumps(dict(status="ERROR", error=repr(exc), traceback=traceback.format_exc()), indent=2)
        )
        traceback.print_exc()
        return 1
    finally:
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    raise SystemExit(main())
