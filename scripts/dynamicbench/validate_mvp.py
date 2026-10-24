#!/usr/bin/env python3
"""Run the physical acceptance cases in one Isaac application, with saved evidence."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))


def main():
    import json
    import traceback

    import numpy as np
    import yaml

    from dynamicbench.cli import launch, parser

    p = parser(collect=True)
    args, launcher, commit = launch(p)
    env = None
    cases = []

    def persist():
        temp = args.output / ".acceptance.tmp"
        temp.write_text(json.dumps(dict(source_commit=commit, cases=cases), indent=2))
        temp.replace(args.output / "acceptance.json")

    try:
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.episode import metadata, run_episode

        env = ConveyorCubeTask(yaml.safe_load(args.config.read_text()), launcher.app, args.seed, 0.03)
        (args.output / "asset_audit.json").write_text(json.dumps(env.asset_audit, indent=2))

        def case(name, test, speed, seed, mode="no_latency"):
            env.speed = speed
            env.reset(seed)
            recorder, metrics = run_episode(
                env, test=test, ticks=100 if test != "expert" else args.ticks, mode=mode, latency=0.25
            )
            entry = recorder.save(args.output, name, metadata(env, metrics, commit), video=not args.no_video)
            arrays = recorder.arrays()
            metrics["episode"] = entry["episode"]
            metrics["passed"] = metrics.get("passed", metrics["success"] if test == "expert" else True)
            cases.append(metrics)
            persist()
            print("ACCEPTANCE " + json.dumps(metrics), flush=True)
            return metrics, arrays, recorder.trace

        # Belt transport must be proven before expert collection begins.
        for speed, name in [(0.0, "belt_static"), (0.03, "belt_03"), (0.06, "belt_06")]:
            result, _, _ = case(name, "belt", speed, args.seed)
            if not result["passed"]:
                return 2
        result, _, _ = case("static_grasp", "expert", 0.0, args.seed)
        if not result["passed"]:
            return 2
        successes = 0
        for i in range(args.max_attempts):
            result, _, _ = case(f"moving_seed{args.seed + i}", "expert", 0.03, args.seed + i)
            successes += int(result["success"])
            if successes >= args.num_success:
                break
            # Stop early on repeated identical grasp failures; retain every failed attempt.
            if i >= 2 and successes == 0:
                return 2
        if successes < args.num_success:
            return 2
        schedule = {}
        for mode in ("no_latency", "sync_hold", "rtc_async"):
            result, arrays, trace = case(f"schedule_{mode}", "schedule", 0.03, args.seed, mode)
            schedule[mode] = (arrays, trace)
            if mode != "no_latency":
                displacement = float(arrays["oracle_debug.cube_pose"][8, 0] - arrays["oracle_debug.cube_pose"][0, 0])
                result["cube_displacement_during_initial_latency"] = displacement
                result["passed"] = displacement > 0.006
            if mode == "rtc_async":
                old = trace[16:24]
                result["old_chunk_executes_during_prediction"] = all(
                    row["chunk_id"] == 0 and not row["hold"] for row in old
                )
                result["actual_robot_moves_during_prediction"] = float(
                    np.linalg.norm(
                        arrays["observation.state.eef10"][24, :3] - arrays["observation.state.eef10"][16, :3]
                    )
                )
                result["passed"] = bool(
                    result["passed"]
                    and result["old_chunk_executes_during_prediction"]
                    and result["actual_robot_moves_during_prediction"] > 1e-4
                )
            persist()
        robot_trajectories = [v[0]["observation.state.eef10"] for v in schedule.values()]
        separation = [
            float(np.max(np.linalg.norm(robot_trajectories[0][:, :3] - x[:, :3], axis=1)))
            for x in robot_trajectories[1:]
        ]
        summary = dict(
            source_commit=commit,
            passed=all(x["passed"] for x in cases) and min(separation) > 1e-4,
            moving_successes=successes,
            moving_attempts=sum(x["test"] == "expert" and x["belt_speed"] == 0.03 for x in cases),
            robot_trajectory_separation_m=separation,
            cases=cases,
        )
        (args.output / "acceptance.json").write_text(json.dumps(summary, indent=2))
        return 0 if summary["passed"] else 2
    except Exception as exc:
        (args.output / "acceptance_error.json").write_text(
            json.dumps(dict(error=repr(exc), traceback=traceback.format_exc(), cases=cases), indent=2)
        )
        traceback.print_exc()
        return 1
    finally:
        if env is not None:
            env.close()
        launcher.app.close()


if __name__ == "__main__":
    raise SystemExit(main())
