#!/usr/bin/env python3
"""Actual PhysX integration entry. No fake backend or synthetic grasp option."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))


def main():
    import argparse
    import json
    import traceback
    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', default='pick_cube_from_conveyor', choices=['pick_cube_from_conveyor'])
    parser.add_argument('--belt-speed', type=float, default=.03)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--ticks', type=int, default=600)
    parser.add_argument('--test', choices=['belt','expert','schedule'], default='expert')
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/mvp')
    parser.add_argument('--config', type=Path, default=ROOT/'dynamicbench/configs/pick_cube_from_conveyor.yaml')
    parser.add_argument('--latency-mode', choices=['no_latency','sync_hold','rtc_async'],default='no_latency')
    parser.add_argument('--latency', type=float, default=.25)
    from isaaclab.app import AppLauncher
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    args.enable_cameras = True
    from env.camera_manager.capture.render_sync import add_zero_delay_kit_args
    add_zero_delay_kit_args(args)
    args.output.mkdir(parents=True,exist_ok=True)
    launcher = AppLauncher(args)
    env = None
    try:
        from dynamicbench.envs.conveyor_cube_task import ConveyorCubeTask
        from dynamicbench.envs.fixed_rate_controller import FixedRateController
        from dynamicbench.experts.conveyor_cube_expert import ConveyorCubeExpert
        settings = yaml.safe_load(args.config.read_text())
        env = ConveyorCubeTask(settings,launcher.app,args.seed,args.belt_speed)
        (args.output/'asset_audit.json').write_text(json.dumps(env.asset_audit,indent=2))
        controller = FixedRateController(env)
        expert = ConveyorCubeExpert(settings,env.belt_top)
        start = controller.observe()
        trace = []
        success,reason = False,None
        for k in range(args.ticks):
            obs = controller.observe()
            if args.test == 'belt':
                action, hold, scheduling = start['eef10'],True,{}
            elif args.test == 'expert':
                action, hold, scheduling = expert.action(obs),False,{}
            else:
                from dynamicbench.runtime.rtc_scheduler import RTCScheduler
                from dynamicbench.runtime.mock_chunk_policy import MockChunkPolicy
                if k == 0:
                    scheduler = RTCScheduler(args.latency_mode,args.latency)
                    policy = MockChunkPolicy()
                action, scheduling = scheduler.tick(k,obs,policy)
                hold = scheduling['hold']
            result = controller.step(action,hold=hold)
            next_obs = result['next_observation']
            record = dict(sim_time=obs['sim_time'],physics_step=obs['physics_step'],control_tick=k,
                phase=expert.phase, robot_actual_pose=obs['eef10'].tolist(),
                commanded_eef=result['action'].tolist(), commanded_q=result['joint_target'].tolist(),
                actual_q=obs['qpos'].tolist(), cube_pose=obs['oracle_debug']['cube_pose'].tolist(),
                finger_contact_forces=obs['oracle_debug']['finger_contact_forces'].tolist(),
                belt_speed=args.belt_speed,control_error=result['control_error'],**scheduling)
            trace.append(record)
            if k%30 == 0:
                print(json.dumps(record),flush=True)
            success,reason = env.outcome(next_obs,k+1)
            if args.test == 'expert' and (success or reason or expert.failure):
                reason = reason or expert.failure
                break
        end = controller.observe()
        import numpy as np
        displacement = end['oracle_debug']['cube_pose'][:3]-start['oracle_debug']['cube_pose'][:3]
        metrics = dict(test=args.test,success=success,failure_reason=reason,
            seed=args.seed,belt_speed=args.belt_speed,control_ticks=len(trace),
            physics_steps=controller.clock.physics_step,sim_time=controller.clock.sim_time,
            cube_displacement=displacement.tolist(),max_cube_z=env.max_cube_z,
            mean_cube_speed=float(displacement[0]/controller.clock.sim_time),
            latency_mode=args.latency_mode,latency_seconds=args.latency,
            real_model_tested=False)
        if args.test=='belt':
            expected = args.belt_speed*controller.clock.sim_time
            metrics['expected_displacement_x'] = expected
            metrics['pass'] = bool(abs(displacement[0]-expected)<max(.01,abs(expected)*.15)
                and abs(displacement[1])<.01 and abs(displacement[2])<.01)
        (args.output/'trace.jsonl').write_text(''.join(json.dumps(t)+'\n' for t in trace))
        (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2))
        print(json.dumps(metrics),flush=True)
        return 0 if (metrics.get('pass',success) or args.test=='schedule') else 2
    except Exception as exc:
        (args.output/'error.json').write_text(json.dumps(dict(status='ERROR',error=repr(exc),traceback=traceback.format_exc()),indent=2))
        traceback.print_exc()
        return 1
    finally:
        if env is not None:
            env.close()
        launcher.app.close()

if __name__=='__main__':
    raise SystemExit(main())
