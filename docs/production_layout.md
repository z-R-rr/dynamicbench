# Official conveyor layout in the production task

The default `pick_cube_from_conveyor.yaml` selects `layout.profile: official`
and `belt.source: robodojo`. `run_mvp`, `collect_expert`, and `evaluate_policy`
all use the same resolver. The render-only audit is no longer the sole path
that restores the original conveyor placement.

The resolver reads the original saved layout
`Assets/Eval_Layout/RoboDojo/arx_x5/0/pick_from_conveyor_by_image_0.json`.
It uses that asset index, scale, position and orientation without fitting the
conveyor into the compact task box. Source paths and SHA-256 values are saved
in episode metadata. With the reference asset this is scale `[1,1,1]`, position
`[-2.08,0.175,0.05]`, WXYZ `[0,0,0,1]`.

The executor measures the spawned Belt mesh's world bounds. Its top defines the
cube's reset height and the success-height reference; its XY bounds define the
conveyor exit test. Metadata support samples provide only a bootstrap height,
not a substitute for the actual transformed geometry. The task still owns
surface velocity on each physics step; the USD's demo objects and render-tick
graph remain disabled. No cube pose/velocity writes occur during execution.

X5 starts from the joint defaults in the original robot configuration, at the
official left base. The official right X5 is instantiated at its configured
base and held with finite joint drives; only the left arm receives task actions.
Head camera, wrist camera, stand, Room, lighting and render parameters keep
their existing official sources. Ground now uses the source mesh, UVs, transform
order, physics material and `material_0564` MDL instead of hiding a gray collider.
Room collision properties remain source-authored.

This remains the single-cube grasp task. The original image-identification
task's basket, photo board and extra target objects are not introduced. During
execution robot joints naturally differ from any arbitrary official video frame.
Piper/Franka retain their task-specific base and pregrasp adaptation; the
conveyor and scene resolver is shared, but X5 home/second-arm restoration applies
only to X5. Their earlier compact-layout success is not official-layout evidence.

Historical compact experiments remain reproducible through an explicit flag:

```bash
bash scripts/dynamicbench/run_isaac61.sh run_mvp --headless \
  --layout-profile compact --belt-source robodojo --test expert \
  --output outputs/compact_reproduction
```

Default official-layout verification commands:

```bash
bash scripts/dynamicbench/run_isaac61.sh run_mvp --headless \
  --device cuda:2 --render-gpu 2 --test expert --seed 42 \
  --output outputs/official_grasp
bash scripts/dynamicbench/run_isaac61.sh collect_expert --headless \
  --device cuda:2 --render-gpu 2 --num-success 3 --max-attempts 3 \
  --seed 42 --save-failures --output datasets/official_layout
bash scripts/dynamicbench/run_isaac61.sh evaluate_policy --headless \
  --device cuda:2 --render-gpu 2 --validate-scheduling --ticks 64 \
  --latency .25 --horizon 32 --prefetch-threshold 16 \
  --output outputs/official_scheduling
```

Choose a currently free GPU before launching. Save new episodes in separate
directories; do not relabel or overwrite compact-layout demonstrations.
