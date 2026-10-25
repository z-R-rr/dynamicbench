# DynamicBench first-stage reproduction

Fork: https://github.com/z-R-rr/dynamicbench
Branch: feat/conveyor-rtc-expert-mvp. Original RoboDojo LICENSE/README retained.
Only the independent entrypoints below use the new timing and scene.

## Isaac Sim 6.1 environment

The conveyor task has an independent Python 3.12 / Isaac Sim 6.1 / Lab 3
adapter. See [the migration record](isaacsim61_migration.md) for the pinned
installer, task commands and current acceptance status. Use
`run_isaac61.sh` for that environment; `run_isaac.sh` continues to use the
currently activated Python. The 6.1 physics suite passed, but the final
cross-reset image validation is still pending after a server connection loss.

## Existing Isaac Sim 5.1 installation

The tested server root is `/home/ubuntu/Workspace/dynamicbench` (no nested repo).
Use the existing Isaac environment, Python 3.11 and Isaac Sim 5.1.0. The report
pins the exact installed Isaac Lab package and checkout. For a fresh server,
follow upstream `bash scripts/install.sh -i` then `bash scripts/init_assets.sh`.
The upstream installer can update dependencies/submodules; the tested server
reuses its existing environment instead of modifying another project checkout.

```bash
git clone -b feat/conveyor-rtc-expert-mvp https://github.com/z-R-rr/dynamicbench.git /home/ubuntu/Workspace/dynamicbench
cd /home/ubuntu/Workspace/dynamicbench
git remote add upstream https://github.com/RoboDojo-Benchmark/RoboDojo.git
source /home/ubuntu/miniconda3/bin/activate RoboDojo
# Either download assets, or link the existing complete server asset tree:
ln -s /home/ubuntu/Workspace/RoboDojo/Assets Assets
# On a fresh machine without that tree, use instead:
# bash scripts/init_assets.sh
bash scripts/dynamicbench/prepare_server.sh
```

prepare_server installs small Python requirements and, when libGLU is absent,
extracts Ubuntu libglu1-mesa/libopengl0 packages under outputs/deps. It does not
need sudo. run_isaac.sh prepends that local library directory.

Required X5 files: Assets/Robots/x5/{ARX.usd,X5A.urdf}; original belt audit:
Assets/Object/RoboDojo/Dynamic/conveyor/{00000,00001}/object.usdz. Full asset
source is HF datasets/RoboDojo-Benchmark/RoboDojo, Assets subtree. Do not commit
assets/datasets/videos. `asset_audit.json` describes what was found.

## Run in PhysX

GPU 2 was selected for rendering after checking utilization. Check current GPU
usage before reproducing; CPU PhysX is deliberate for surface velocity support.
The final reproduced run separated CUDA and Vulkan ordinals to avoid the busy
physical GPU0 while rendering on the least-used physical GPU2. Check usage first:

```bash
nvidia-smi --query-gpu=index,memory.used,memory.free --format=csv
export CUDA_VISIBLE_DEVICES=2,1
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
# Physical GPU2 is CUDA0 under this explicit mask. The second visible GPU has
# no policy/physics worker; it is not used to make a multi-GPU experiment.
gpu=(--headless --device cuda:0 --render-gpu 2 --kit_args="--/validate/iommu/enabled=false --/validate/p2p/enabled=false --/plugins/carb.tasking.plugin/threadCount=8")
```

These flags only configure this process and skip startup cross-device self-tests;
they do not change system IOMMU. GPU selection must be refreshed on another host.
Without a CUDA mask, --device cuda:2 selects physical GPU2 directly, but the
installed SDK may also initialize busy GPU0. --render-gpu independently names
the physical Vulkan ordinal; this avoids the incompatible enumeration produced
by masking to CUDA0 while also implicitly selecting physical rendering GPU0.

These commands are actual integration commands, not mocks. Use a fresh output
folder per run; existing episodes are never overwritten.

```bash
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
# Full suite: transport, static grasp, three moving demos, latency comparisons.
bash scripts/dynamicbench/run_isaac.sh validate_mvp "${gpu[@]}" --num-success 3 --max-attempts 10 --seed 42 --output outputs/acceptance_repro
bash scripts/dynamicbench/run_isaac.sh run_mvp "${gpu[@]}" --test belt --ticks 100 --belt-speed .03 --seed 42 --output outputs/belt03
bash scripts/dynamicbench/run_isaac.sh run_mvp "${gpu[@]}" --test expert --belt-speed 0 --seed 42 --output outputs/static42
bash scripts/dynamicbench/run_isaac.sh run_mvp "${gpu[@]}" --test expert --belt-speed .03 --seed 42 --output outputs/moving42
bash scripts/dynamicbench/run_isaac.sh collect_expert "${gpu[@]}" --task pick_cube_from_conveyor --num-success 3 --max-attempts 10 --belt-speed .03 --seed 42 --save-failures --output datasets/conveyor_cube_v0
python scripts/dynamicbench/inspect_dataset.py datasets/conveyor_cube_v0
python scripts/dynamicbench/inspect_dataset.py datasets/conveyor_cube_v0/episode_seed42 --video outputs/preview42.mp4
for mode in no_latency sync_hold rtc_async; do
  bash scripts/dynamicbench/run_isaac.sh run_mvp "${gpu[@]}" --test schedule --ticks 100 --belt-speed .03 --seed 42 --latency .25 --latency-mode "$mode" --output "outputs/schedule_$mode"
done
python -m pytest dynamicbench/tests -q
```

Expert collection is currently no_latency. The latency entry uses a deterministic
present-observation mock, H=32/stride=16. Unsupported delayed expert collection
is rejected explicitly; it is not a future-GT oracle. Fixed latency is simulated,
not measured GPU inference. Holds use finite drives. 250 ms -> ceil(7.5)=8 ticks
=266.667 ms, quantization +16.667 ms. RTC action i maps to request_tick+i;
returning discards elapsed prefix once. sync_hold reuses the entire stale chunk
after its wait, unlike LIBERO-Dynamic's sub-tick continuous hold latency.

## Inspect evidence

Each complete episode has trajectory.npz (all raw RGB/state/command frames),
metadata.json, trace.jsonl and head.mp4. `metrics.json` or collection_metrics.json
states actual attempts/successes/failures and source commit. Reader validates
N actions/N+1 observations, timestamps, real-vs-commanded fields and success
lift/contact evidence. Failed runs return a nonzero status and are never
labelled successful. See results/mvp_report.md for actual PASS/FAIL/NOT RUN.

Unit tests can run without Isaac Sim in a Python 3.11 virtualenv:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r scripts/dynamicbench/requirements.txt
python -m pytest dynamicbench/tests -q
```


## Scene and startup boundaries

Default belt source is `primitive`, with a real collidable kinematic surface
using PhysxSurfaceVelocityAPI. Cube writes occur only in reset; all transport
and grasp motion is contact-driven. Speeds 0/.03/.06 were measured for transport;
expert grasp was measured only at 0 and .03. Success requires cube-bottom
clearance >.05 m plus bilateral cube-filtered contacts for 15 ticks (0.5 s).
Three seeds are a small deterministic MVP check, not a general success estimate.

`--belt-source robodojo` selects the original conveyor USD. It passed the same
10-case live acceptance (0/.03/.06 transport, static grasp, 3/3 moving grasps at
.03, and all three fixed-delay modes) after the exact cube-path fix. Details and
reproduction command: docs/original_usd_fix.md. The default remains primitive
so existing datasets/configurations retain their scene semantics.

The original GPU0 OOM path was isolated with the CUDA/Vulkan mapping above.
Use --render-gpu only with the physical index shown by nvidia-smi; --device
uses the CUDA index after remapping. No driver, BIOS, IOMMU or other user job
was changed. AppLauncher uses a narrow local subclass to set active_gpu after
Isaac Lab would otherwise derive it from the remapped CUDA ordinal. This was
tested against the pinned Isaac Lab checkout.

Head-camera/robot placement is read directly from RoboDojo's original configs:
layout.robot_config=env_cfg/robot/dual_x5.yml, robot_index=0;
layout.camera_config=env_cfg/camera/camera_config.yml, head_camera=cam_head.
Gemini_345Lg lens values/resolution come from env_cfg/camera/template.py.
Single-arm placement is the original left-arm base, normalized quaternion.
The expert-specific reset joint pose remains necessary for this task. Source
config hashes and resolved settings are saved with each episode.

Both real head and wrist arrays are stored. To export the wrist preview:

```bash
python scripts/dynamicbench/inspect_dataset.py outputs/acceptance_repro/moving_seed42 --camera wrist --video outputs/wrist42.mp4
```

A native Kit shutdown can stall after an extension failure. Bound new runs with
`timeout -k 10s 1200s` and preserve run.log/exit_status.json. Stop only the exact
owned test process after verifying its command. Exit status alone is not
acceptance: check acceptance.json/collection_metrics.json and strict readback.


## Native fixtures and contact-anchored lifting (2026-10-25)

Current defaults: scene.source=robodojo and lift_xy_mode=contact_anchor.
First contact latches the world EEF reference, three consecutive bilateral
contacts trigger vertical lift, and contact-to-end XY excursion is validated.
The lift_tcp_frame setting applies only to the legacy track_cube diagnostic.

Room/Background settings are read from the original conveyor scene. Native
Room, Background and Light components create the render fixtures. Room
collisions are disabled; the existing physical ground remains hidden from RGB.
Required assets also include Assets/Room/Simple_Room/*.usd and
Assets/Background/brown_photostudio_02_4k.hdr (part of the existing asset tree).

Use --lift-xy-mode track_cube --lift-tcp-frame actual_pinch to reproduce the
previous controller; it still chases cube velocity during close_gripper. Use
--lift-xy-mode fixed for the older fixed-lift baseline, which shares that closing
problem. These are diagnostics, not the recommended expert. --scene-source
minimal switches only the fixtures. --compare-lift adds three legacy fixed
reference episodes to the ten core acceptance cases.

See docs/contact_anchor.md for the 5.1 reproduction and motion evidence.
The subsequent 6.1 installation, adapter and validation status are recorded in
docs/isaacsim61_migration.md. The assessment page is retained as historical
upgrade planning.
