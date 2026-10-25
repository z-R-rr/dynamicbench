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
path retains its asynchronous fence. The synchronous 6.1 change needs the final
live reset-image check described below before dataset acceptance.

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

## Measured status and remaining acceptance (2026-10-25)

The isolated installation and the native conveyor task ran on the server's
GPU 1, chosen from all four GPUs by free memory at launch. The driver remains
580.95.05. The original USD belt, original scene fixtures and both cameras
ran in 6.1. At source `a12761cf63a5c61b2c79edb719bc1b23b65fe605`,
`/home/ubuntu/Workspace/dynamicbench/outputs/isaacsim61_acceptance01`
completed all ten physics cases:

| Case | Measured result |
| --- | --- |
| Belt 0 / .03 / .06 m/s | 3/3 transport checks passed; .03 measured .0299954 m/s |
| Static grasp | Passed, 77 control ticks |
| Moving seeds 42 / 43 / 44 at .03 m/s | 3/3 attempts succeeded, 82 / 81 / 81 ticks |
| no_latency / sync_hold / rtc_async | 3/3 scheduling checks passed with the mock policy |
| Separate primitive-belt seed42 | Passed, 82 ticks at source `387fe825124cec6340be751af41fadc9b903a28a` |

Actual PhysX callback counts and image-capture-without-physics assertions passed
during those runs. Three specified seeds are a deterministic smoke test, not a
general success-rate estimate. No physical robot or real asynchronous policy
inference was tested. The rest of RoboDojo's upstream task suite is outside this
adapter's validation scope.

**The 6.1 dataset is not yet accepted.** Inspection of the raw RGB found a stale
first head image after a cross-episode reset. In moving_seed42, the initial
state projects the cube to (210.53, 207.28) pixels while the image's orange-cube
centroid is (187.97, 174.35), a 39.92-pixel discrepancy. The following four
frames are within 4.65–4.77 pixels. The original acceptance01 data is retained
as diagnostic evidence and must not be exported as the final training set.

Commit `349a01977172c8b6b4b508084fafe870be2c1443` enables the synchronous
6.1 renderer settings. Its contract CI passed, but the server-side fetch failed
with a GitHub TLS interruption before `isaacsim61_acceptance02` could start.
The server then became unreachable at its configured address 10.170.157.74.
The last confirmed server checkout is a12761c. Live validation of the camera
change and final local/server commit parity remain pending connectivity.

`audit_reset_rgb.py` adds a no-simulator geometric/color check of every episode's
first head image, with a ten-pixel tolerance. It is specific to this unoccluded
orange-cube reset scene; it catches the observed cross-reset defect but cannot
prove sub-tick image freshness. Its regression tests reject a cube in a previous
lift position and an image with no visible cube. All 40 local unit tests and
Ruff lint checks pass after this addition.

After reconnecting, fetch and fast-forward the branch, rerun `validate_mvp` in
a fresh output directory, run strict readback and `audit_reset_rgb.py` on all
ten episodes, then inspect both cameras around reset and export the three
accepted moving episodes. Verify the actual runtime/source metadata rather
than reusing the results above for a newer commit.

One first full-scene launch crashed natively in PhysX `start_simulation` after
mesh-cooking warnings. Subsequent room-only, belt-only and full-scene launches
succeeded; a cache-warmup explanation is an inference, not an established root
cause. Keep the initial crash log and check future cold starts before claiming
startup reliability. Cached startup was about eleven seconds, while the first
RTX warmup took about twenty minutes. The driver was not changed to obtain
the successful runs.
