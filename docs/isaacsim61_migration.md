# DynamicBench Isaac Sim 6.1 migration

The independent conveyor task now has a native Lab 3 PhysX adapter. This does
not migrate every upstream RoboDojo task. The existing 5.1 environment and
upstream environment wrapper remain available for rollback.

## Pinned environment

- Isaac Sim `6.1.0.0`, Python `3.12.15`.
- Isaac Lab release `v3.0.0-EA`, commit
  `ae37b028ea415c91ea2bc32609efcd759ed2b974`.
- Package versions are `isaaclab==17.0.2`, `isaaclab-physx==6.0.0`;
  these package numbers differ from the Lab release label.
- PyTorch `2.11.0+cu128`, Warp `1.16.0`, uv `0.13.5`.
- Official `uv.lock` SHA-256:
  `98570361a2f86abcfdebfda35f853da0945a8ccbe422f51d5fffaf1936d478a4`.

The version combination follows the [official release](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-EA).
The [migration guide](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/migration/migrating_to_isaaclab_3-0.html)
explains the quaternion, ProxyArray, backend, and indexed-write changes.
The pinned source and frozen lock file are authoritative for installation.

From this repository's root, with Conda available:

```bash
bash scripts/dynamicbench/prepare_isaac61.sh
```

Installation is isolated under `outputs/envs/dynamicbench61`, with the matching
vendor source in `outputs/deps/IsaacLab61`. `outputs/envs/isaacsim61` is only the
Python/uv bootstrap. Assets continue to use the existing `Assets` tree.
Neither the old Conda environment nor the GPU driver is upgraded by this script.

## Adapter behavior

`dynamicbench/envs/base_env61.py` owns `SimulationContext` and `InteractiveScene`
directly. Physics uses CPU PhysX at 300 Hz; rendering uses the selected RTX GPU.
Each control tick contains ten physics steps. Physics callbacks count actual
steps, and an unexpected count is fatal. Camera renders explicitly pump Kit
through the Lab renderer helper with physics disabled. Capturing both images
must leave the observed physics-step count unchanged.
The 6.1 CLI enables both renderer/hydra `waitIdle` settings, while the old 5.1
path retains its asynchronous fence. The synchronous 6.1 change passed the live reset-image checks described below.

`runtime.py` converts WXYZ configuration quaternions into Lab's XYZW and converts
observed poses back to WXYZ. EEF10 rotation-6D, trajectory fields and existing
dataset conventions are unchanged. ProxyArray reads use `.torch`; state writes
use the indexed APIs and remain confined to reset. Implicit joint drives and
contact-anchor expert logic are shared with 5.1. PhysX contact values are filtered
normal forces, rather than an inferred total normal-plus-friction force.

The original belt uses `PhysxSurfaceVelocityAPI`, with its source animation graph
disabled only on the spawned instance. Original asset files remain unchanged.
Room, Background, Light, robot and camera configurations are reused. Sim 6.1
removed `isaacsim.core.utils` and the old single-prim wrappers, so `fixtures61.py`
authors the fixed room/background/RectLight configuration through USD. Sim 5.1
continues to instantiate the original RoboDojo component classes. The 6.1 fixture
adapter deliberately rejects randomized ranges and unsupported light types.
Newton is not selected for this task. Camera rendering uses the new RTX backend
with DLAA, DL denoising and four direct-light samples per pixel: retaining the old
TAA setting alone produced visibly noisy images in 6.1. Layout and camera
extrinsics are preserved; pixel equality between engines is not claimed.

## Run and rollback

Choose the currently least-used GPU using `nvidia-smi`. Without a CUDA mask, use
the same physical index for `--device cuda:N` and `--render-gpu N`:

```bash
bash scripts/dynamicbench/run_isaac61.sh validate_mvp \
  --headless --device cuda:1 --render-gpu 1 --belt-source robodojo \
  --num-success 3 --max-attempts 3 --seed 42 --output outputs/acceptance61_repro \
  --kit_args="--/plugins/carb.tasking.plugin/threadCount=8 --/crashreporter/preserveDump=true"

# Individual transport and grasp runs, then independent expert collection:
bash scripts/dynamicbench/run_isaac61.sh run_mvp \
  --headless --device cuda:1 --render-gpu 1 --belt-source robodojo \
  --test belt --ticks 100 --belt-speed .03 --output outputs/belt61_repro
bash scripts/dynamicbench/run_isaac61.sh run_mvp \
  --headless --device cuda:1 --render-gpu 1 --belt-source robodojo \
  --test expert --belt-speed .03 --seed 42 --output outputs/grasp61_repro
bash scripts/dynamicbench/run_isaac61.sh collect_expert \
  --headless --device cuda:1 --render-gpu 1 --belt-source robodojo \
  --num-success 3 --max-attempts 3 --seed 42 --save-failures \
  --output datasets/conveyor_cube_61_repro

py=outputs/envs/dynamicbench61/bin/python
"$py" scripts/dynamicbench/inspect_dataset.py outputs/acceptance61_repro
"$py" scripts/dynamicbench/audit_reset_rgb.py outputs/acceptance61_repro \
  --output outputs/acceptance61_repro/reset_image_audit.json
"$py" -m pytest dynamicbench/tests -q
```

Use a fresh output directory each time. For rollback, activate the original
RoboDojo Conda environment and use `run_isaac.sh` with the same task arguments.
Metadata records the actual runtime versions as well as the source commit.
A successful install or CPU test is not a successful simulator acceptance.

## Measured acceptance (2026-10-25)

The completed suite uses source `b238ca5aabcbfaff08aa3f1cbc4ff7add37a71be`.
The server run is
`/home/ubuntu/Workspace/dynamicbench/outputs/isaacsim61_acceptance02`.
It was launched after a server reboot, using GPU 1 and the unchanged
580.95.05 driver. The original USD belt, RoboDojo room fixtures and both
cameras ran successfully. Exit status is zero.

| Case | Measured result |
| --- | --- |
| Belt 0 / .03 / .06 m/s | 3/3 transport checks passed; .03 measured .0299954 m/s |
| Static grasp | Passed, 77 control ticks |
| Moving seeds 42 / 43 / 44 at .03 m/s | 3/3 attempts succeeded, 82 / 81 / 81 ticks |
| no_latency / sync_hold / rtc_async | 3/3 scheduling checks passed with the mock policy |
| Separate primitive belt with RoboDojo room, seed42 | Passed, 82 ticks at the same b238ca5 source |
| Strict dataset readback / head-video frame counts | 10/10 original-USD episodes passed |
| Cross-reset first-head-image audit | 10/10 passed, centroid errors 1.96–5.28 pixels |
| 5.1 rollback, original USD + room, seed42 | Passed at b238ca5, 82 ticks; state/RGB/video readback passed |
| Local and server unit tests | 40/40 passed; contract CI also passed on Python 3.11 and 3.12 |

Actual PhysX callback counts and capture-without-physics assertions passed.
On the three moving seeds, first-contact-to-terminal EEF XY peak excursion is
0.342–0.353 mm, commanded XY excursion is zero, and cube XY peak excursion is
7.70–7.88 mm. Bilateral contact persists through the required final 15 ticks
above the success height. This is a small deterministic smoke test; three
specified seeds do not establish a general success-rate estimate.

The corrected three moving demonstrations were exported from the acceptance
suite with source/runtime metadata, raw head/wrist RGB, states, commands,
trace and manifest. This export is not an additional independent collection
experiment. The ZIP CRC and per-trajectory SHA-256 values were checked.

Machine-readable results are `results/isaacsim61_acceptance.json`,
`results/isaacsim61_readback.json`, `results/isaacsim61_reset_images.json`,
`results/isaacsim61_primitive.json` and `results/isaacsim51_rollback.json`.
Local delivery root:
`/Users/riri/Documents/Codex/2026-10-23/files-mentioned-by-the-user-dynamicbench/outputs/isaacsim61`:

- `acceptance_final/`: all ten complete original-USD episodes and logs.
- `primitive_final/`: the separate successful default-belt check.
- `rollback51/`: the successful 5.1 rollback grasp and strict readback.
- `dataset/manifest.jsonl` and `dataset_isaacsim61.zip`: three accepted moving demonstrations.
- `acceptance_final/moving_seed42/{head,wrist}.mp4`: directly recorded/exported camera previews.
- `head_frames_final.png`, `wrist_frames_final.png`: reset and grasp contact sheets.
- `setup/`: installed package freeze and setup log.

### Reset image defect and correction

At a12761c, physics passed but raw RGB inspection found the previous episode's
last frame in moving_seed42's first image after reset. Its cube-state projection
was (210.53, 207.28) pixels while the orange-cube centroid was (187.97, 174.35),
a 39.92-pixel discrepancy. The following four frames were within 4.65–4.77 pixels.
That run, `isaacsim61_acceptance01`, is retained as diagnostic evidence and is
excluded from the accepted training export.

The old 5.1 asynchronous render fence did not suffice for Lab 3. Commit 349a019
enables synchronous renderer/hydra delivery for 6.1, retaining the old settings
for 5.1. After connectivity was restored, b238ca5 was synchronized and the full
suite rerun. Moving_seed42's first-image discrepancy is now 4.95 pixels; its first
two main and wrist images visually show the reset scene rather than a held cube.
All ten first-image audits pass. `audit_reset_rgb.py` compares the unoccluded
orange cube's centroid with its geometric projection using a ten-pixel tolerance.
It detects the observed cross-reset defect; it cannot establish sub-tick camera
freshness during steady motion. Unit regressions reject a cube at a previous
lift position and a frame with no visible cube.

### Scope and startup observations

This migration covers the independent conveyor task and its entrypoints, not
all upstream RoboDojo tasks. No physical robot, real asynchronous model
inference, Newton backend or .06 m/s grasp was tested. The original 5.1 Conda
environment, upstream task/config/environment sources, assets and driver are
preserved. Fixed Room/Background/RectLight configurations are supported by the
6.1 fixture adapter; randomized fixture ranges deliberately raise an error.

One early full-scene start crashed natively in PhysX `start_simulation` after
mesh-cooking warnings. Later room-only, belt-only, full-scene and post-reboot
full-scene launches succeeded. The cause remains unproven; a cooking-cache
explanation is an inference. Keep the initial crash evidence and do not infer
broad cold-start reliability from these runs. Cached application startup was
about eleven seconds before reboot; post-reboot startup was about 21 seconds.
The first RTX cache warmup took about twenty minutes. No driver change was
needed for the successful launches.
