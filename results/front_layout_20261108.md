# Front installation on the official conveyor — 2026-11-08

Piper and Franka now inherit the active X5 front mounting slot instead of
overriding it with a side/end placement. The official production-layout branch
was merged without changing its conveyor, camera or rendering implementations.

Merge: `0b047bd8ee7c6f5e1b40e121f998d281393692b7`.
Accepted runtime source: `f47141dbddb257963bda2cfa92e97c50a75f9431`.
Isaac Sim 6.1.0.0, CPU PhysX at 300 Hz, control at 30 Hz; RTX GPU 3 for
Franka expert collection and GPU 2 for Piper and both scheduling validations.
Devices were selected when they had the least occupied VRAM.

## Layout and configuration invariants

Both robots inherit `env_cfg/robot/dual_x5.yml`, index 0: position
`[-0.3,-0.45,0.765]`, WXYZ `[0.7071067812,0,0,0.7071067812]`.
Their world grasp orientations rotate with that mount, and their own URDFs
provide the kinematics, joints, TCP offsets and gripper limits. Pregrasp IK is
performed after production geometry is resolved. A regression test changes the
compact-only height to 20 m and verifies it cannot affect the official pose.

The official conveyor remains at source root `[-2.08,0.175,0.05]`, WXYZ
`[0,0,0,1]`, scale `[1,1,1]`. The spawned Belt surface remains at
`0.7622119448 m`. Head-camera pose/lens, per-robot wrist local mounting/lens,
cube settings, expert gates and render settings are unchanged. The full scene
and belt metadata match the independently accepted official X5 run after only
normalizing checkout-root paths. Both checkouts link to the same source assets.
Wrist world poses follow the arms as intended. X5 retains the official passive
right arm; Piper/Franka are single active-arm adapters.

## Physical expert validation

Original conveyor USD, 0.03 m/s, seeds 42–44, exactly three attempts each:

| Robot | Seed | Control ticks | Peak EEF XY after contact (mm) | Peak cube XY after contact (mm) | Result |
|---|---:|---:|---:|---:|---|
| franka | 42 | 102 | 3.248 | 4.625 | PASS |
| franka | 43 | 101 | 3.262 | 4.697 | PASS |
| franka | 44 | 101 | 3.309 | 5.010 | PASS |
| piper | 42 | 105 | 2.157 | 8.684 | PASS |
| piper | 43 | 105 | 2.345 | 7.258 | PASS |
| piper | 44 | 105 | 1.226 | 7.009 | PASS |

All six episodes passed the original lift height, bilateral contact, hold
duration and post-contact motion gates. No success threshold was relaxed.
Strict NPZ readback and decoded video frame counts passed; seed-42 head/wrist
reset, contact and terminal frames were visually inspected for each adapter.
The runtime render audit passed all 18 settings and preserved room collisions.

## Scheduling and code validation

Eight additional simulator cases passed: no latency, synchronous hold,
asynchronous queue and RTC for each changed embodiment, 64 actions per case,
250 ms requested latency, horizon 32 and prefetch threshold 16. Sync used
790 physics steps, including two 75-step waits; other modes used 640 steps.
Trace timestamps, finite joint-target holds, eight initial underflow ticks,
24 old-queue executions during pending async/RTC requests, dropped prefixes,
video duration and belt travel were independently checked.

63 CPU tests and Ruff passed. The independent official X5 three-seed grasp and
four-mode scheduling results remain in
[the production report](../docs/production_layout_validation_20261108.md), with
their own source revision; they are not relabeled as new adapter results.

These are small oracle-expert and deterministic-mock checks, not a robustness
estimate or a learned-model evaluation. No compatible checkpoint was evaluated.

## Reproduction and artifacts

Run from the repository with the original RoboDojo assets and Piper assets
available. Select a currently available GPU and always use a new output folder:

```bash
bash scripts/dynamicbench/run_isaac61.sh collect_expert --headless \
  --device cuda:2 --render-gpu 2 --embodiment piper \
  --num-success 3 --max-attempts 3 --ticks 300 --seed 42 --save-failures \
  --output outputs/piper_front_new
bash scripts/dynamicbench/run_isaac61.sh evaluate_policy --headless \
  --device cuda:2 --render-gpu 2 --embodiment piper \
  --validate-scheduling --ticks 64 --latency .25 --horizon 32 \
  --prefetch-threshold 16 --output outputs/piper_front_schedule_new
```

Use `--embodiment franka` for Franka. The default layout is official.
Accepted remote data is in `/home/ubuntu/Workspace/dynamicbench-front-layout-20261108/outputs/`:
`{franka,piper}_front_official_v2`, `franka_front_schedule_v3`, and
`piper_front_schedule_v2`.
Local full data/previews are in the chat workspace's
`outputs/front_layout_official_20261108`.

Exact per-episode metrics, configuration comparisons, trajectory hashes and
artifact hashes: [front_layout_20261108.json](front_layout_20261108.json).
Earlier compact front tests at `51796c6` are retained separately; they do not
establish official-layout acceptance. The initial official integration probe
was stopped to correct initialization order. A GPU 0 compact Piper attempt
was stopped during shader compilation before physical testing; it is not a
grasp failure or training demonstration. The first Franka scheduling launch on
GPU 3 was also stopped during driver shader compilation and rerun on the newly
free GPU 2, without changing code or evaluation parameters.
