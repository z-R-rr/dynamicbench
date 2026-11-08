# Conveyor object configuration

The production task accepts one dynamic rigid body. `object` replaces the old
`cube` block; the task, cameras, conveyor layout, robot adapter and scheduler
are selected independently. The existing task name and exact `/World/envs/env_0/Cube`
prim path stay stable for existing scripts and contact subscriptions.

```yaml
object:
  id: yellow_cube_35mm
  source: primitive
  shape: box
  size: [0.035, 0.035, 0.035]  # full dimensions, meters
  mass: 0.05                  # kg
  friction: 1.5               # static and dynamic
  color: [1.0, 0.55, 0.02]   # optional RGB in [0,1]
  initial_xy: [-0.25, -0.02] # rigid-body origin, world meters
  initial_jitter: 0.005       # independent uniform +/- jitter per XY axis
  initial_quaternion_wxyz: [1, 0, 0, 0]
  spawn_clearance: 0.002
  grasp:
    position: [0, 0, 0.00175]
    opening: 1.0
```

`shape: cylinder` uses `radius` and full `height`, with its local axis along Z.
`shape: sphere` uses `radius`. Use `size` only for boxes. Primitive dimensions
already express their final size; they do not take a separate `scale`.

Pass an object-only YAML to any production entrypoint:

```bash
bash scripts/dynamicbench/run_isaac61.sh run_mvp --headless \
  --device cuda:0 --render-gpu 0 --test expert \
  --object-config dynamicbench/configs/objects/cylinder.yaml \
  --output outputs/cylinder_new_run
```

The same argument works with `collect_expert` and `evaluate_policy`. Select an
idle GPU and a fresh output directory. The override file must contain only the
top-level `object` mapping; it replaces the whole object specification.
Provided profiles are `cylinder.yaml`, `sphere.yaml` and `robodojo_block.yaml`.

## Available local assets

The server inventory checked on 2026-11-08 contains 136 `Rigid` categories and
95 `Clutter` categories under `Assets/Object/RoboDojo`. Category counts do not
mean that every asset has passed the loader or grasp acceptance. Sample assets
from the following paths have a single rigid-body default prim, colliders and
no joints/runtime graphs/physics scenes:

| Path below `Rigid` | Raw geometry size XYZ (m), before configured scale |
| --- | --- |
| `bottle/00000/object.usdz` | 0.063, 0.063, 0.245 |
| `can/00000/object.usdz` | 0.0494, 0.0494, 0.1380 |
| `cup/00001/object.usdz` | 0.072, 0.072, 0.072 |
| `mug/00000/object.usdz` | 0.1457, 0.1155, 0.0747 |
| `bowl/00001/object.usdz` | 0.110, 0.110, 0.060 |
| `hammer/00000/object.usdz` | 0.0587, 0.1588, 0.0145 |
| `toy_car/00000/object.usdz` | 0.0751, 0.0457, 0.0427 |
| `basket_grasp/00002/object.usdz` | 0.3601, 0.2419, 0.2186 |

These are structural candidates, not additional grasp-tested profiles. Cups,
bowls and baskets can require rim/handle grasp points and appropriate hollow
collision geometry. `Clutter` assets must be screened individually. Existing
`Articulation`, `Garment` and `Fluid` groups require different task adapters.
User-provided meshes must first be exported to USD with the single-body and
collision structure described below; OBJ/GLB files are not loaded directly.

## USD assets

```yaml
object:
  id: robodojo_block_00020
  source: usd
  usd_path: Assets/Object/RoboDojo/Rigid/block/00020/object.usdz
  scale: [1, 1, 1]
  mass: 0.05
  friction: 1.5
  initial_xy: [-0.25, -0.02]
  grasp:
    position: [0, 0, 0.0015]
```

Relative `usd_path` is relative to the repository root, not to the override
file. Absolute paths also work. The USD must have exactly one rigid body,
authored on its default prim, and authored colliders. Assets with joints,
multiple rigid bodies, runtime graphs, or embedded physics scenes are rejected.
Deformable objects and articulated containers require a separate task adapter.
Omitting `color` preserves the USD's visual material. Mass, friction, gravity,
solver parameters and contact offsets use the task's finite physical settings.
The loader does not repair collision meshes or infer an object's grasp points.
USD coordinates multiplied by `scale` are interpreted in meters; unit metadata
does not automatically rescale a referenced asset. Check its resolved bounds.

## Geometry and grasp conventions

Reset height accounts for the initial orientation and the object's geometry,
including an offset USD origin. Boxes, spheres and cylinders use their analytic
support extents. USDs use conservative local geometry bounds, transformed with
the live rigid-body orientation. Bounds need not coincide with the lowest mesh
vertex for an irregular rotated object; this conservative test can delay success.

`grasp.position` is a point in the **scaled rigid body's local frame**, in meters.
It is rotated and translated by the current body pose. It is not rescaled again
by `scale`. With no explicit point, primitives use their center (boxes retain
the old 5% height offset); USDs use their local bounds center.
Optional `grasp.quaternion_wxyz` specifies the EEF orientation relative to the
object. When omitted, the robot adapter's normal grasp orientation is used.
`opening` uses the existing normalized gripper command: -1 closed, +1 open.
It is not a distance in meters.

Approach height clears the object's top. After contact the default expert still
latches the measured world EEF pose and lifts vertically, with sufficient
clearance for the object's bottom. It does not chase a slipping object.
Success requires bottom clearance plus sustained bilateral finger contact.
The implementation only writes object state during reset; conveyor transport
and grasp motion remain physical. New geometry can require a different grasp
point/orientation; configuration support does not guarantee grasp success.

## Data and compatibility

Old task YAMLs containing `cube` are normalized at load time. Supplying both
`cube` and `object` in the same task is an error; an explicit `--object-config`
replaces either. Old recorded demonstrations remain readable without edits.

New metadata stores the normalized `object` configuration, schema
`dynamicbench.object.v1`, geometry, and the USD/available dependency SHA-256s.
The manifest records `object_id`. Canonical debug columns include `object_pose`,
`object_velocity`, `object_bottom_z`, `object_top_z` and `object_grasp_position`.
The strict reader recomputes bottom clearance from geometry and recorded poses.
Existing `cube_*` debug columns and `cube_displacement` metrics remain aliases
for downstream diagnostics, including non-cube objects. `cube_size` is emitted
only for an equal-sided box. New consumers should read `object` metadata and
`object_displacement`. Privileged object columns are not added to model inputs.

Timing schemas stay unchanged: expert data has uniform 30 Hz actions; evaluation
data retains explicit physical waiting intervals. Never relabel old cube data
as a different object; collect into a new directory for each asset/configuration.
The old HSV-based yellow-cube reset-image audit is still cube-specific; it is
not a generic object detector for the new profiles.

## Validation on 2026-11-08

Runtime source: `3c82e804aa436547c4008619c264c47bf217a60c`. Isaac Sim 6.1,
CPU PhysX 300 Hz, control 30 Hz, GPU 0 RTX, official X5 conveyor layout.
Each object below passed one seed-42 oracle grasp, bilateral contact/height
checks, strict trajectory readback and video frame-count checks:

| Configuration | Actions | Simulated duration |
| --- | ---: | ---: |
| Default yellow box | 112 | 3.7333 s |
| Cylinder profile | 117 | 3.9000 s |
| RoboDojo block 00020 USD | 113 | 3.7667 s |
| Sphere profile | 109 | 3.6333 s |

The cylinder's four mock scheduler modes also passed: 64 actions; 640 physics
steps and 0.063584 m displacement for no-latency/async/RTC; 790 steps and
0.078517 m for sync, including two 75-step waits. This is not learned-policy
or RTC-conditioned model validation. Other embodiments and arbitrary assets
need their own grasp validation.

Local checks: 76 unit tests, Ruff and diff checks passed. Three old official
layout cube episodes were read back unchanged. Full metrics and SHA-256s for
53 artifacts are in [the validation report](../results/object_config_validation_20261108.json).
Server artifacts: `/home/ubuntu/Workspace/dynamicbench-object-config-20261108/outputs`.
Local artifacts: `/Users/riri/Documents/Codex/2026-10-23/files-mentioned-by-the-user-dynamicbench/outputs/object_config_20261108`.
The local `objects_grasped.png` shows final wrist frames for all four profiles.
