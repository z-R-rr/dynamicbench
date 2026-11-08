# Conveyor task embodiments

Select `--embodiment x5|franka|piper` on `run_mvp`, `collect_expert`, or
`evaluate_policy`. The default remains X5. Belt, cube, room, lights and head
camera stay the same; robot-specific root placement and pregrasp IK change.

| Adapter | Assets | Arm / fingers | Action reference | TCP direction |
|---|---|---|---|---|
| X5 | RoboDojo `Assets/Robots/x5` | 6 / 2 | world `link6` | local +X, source bias |
| Franka | RoboDojo `Assets/Robots/franka` | 7 / 2 | world `panda_hand` | local +Z, source bias |
| Piper | RoboTwin `assets/embodiments/piper` | 6 / 2 | world `link6` | local +Z, source bias |

EEF10 always means absolute world XYZ, first two rotation columns, and measured
gripper opening in [-1,1]. The link name is part of metadata; equal vector
dimensions do not make X5, Piper and Franka checkpoints interchangeable.
The old `tcp_offset_link6` settings key is retained for expert compatibility;
for new adapters it is the offset in their declared `eef_link` frame.

Joint/finger names, gripper travel and TCP distance are read from each source
config and URDF. URDF FK includes fixed links after the actuated joints, notably
Franka's link8 and hand transform. Every simulator reset checks this FK against
the USD articulation before accepting an episode. No robot state writes occur
outside reset; actions go through bounded IK and finite joint drives.

Franka and Piper task roots are [-0.65,-0.02,0.765] with identity orientation.
Pregrasp joints are solved deterministically from each URDF. Franka approaches
vertically. Piper uses a 30-degree inclined approach because its joint5 limit
prevents the selected vertical pregrasp; the jaw yaw avoids the joint6 wrap seam.
These are task placements, not claims of an official RoboDojo conveyor layout.

X5 keeps its official asset camera mount. Franka and Piper use an explicitly
task-mounted D435 at the declared EEF, local position [.05,0,.025], WXYZ
[0,1,0,0]. This is not presented as an official wrist-camera calibration.

Piper assets are discovered in the sibling RoboTwin checkout or supplied via
`--piper-asset-root /absolute/path/to/piper`. Conversion uses Isaac Lab's native
URDF importer into `outputs/assets/piper/<urdf-hash>/piper.usd`; source assets
are not edited. Fixed links are retained; both finger joints are driven.
Generated USD and large assets are not committed.

```bash
bash scripts/dynamicbench/run_isaac61.sh run_mvp --headless \
  --device cuda:3 --render-gpu 3 --embodiment franka --belt-source robodojo \
  --test expert --output outputs/franka_grasp
bash scripts/dynamicbench/run_isaac61.sh evaluate_policy --headless \
  --device cuda:3 --render-gpu 3 --embodiment piper --belt-source robodojo \
  --validate-scheduling --ticks 64 --output outputs/piper_scheduling
```

See the dated results report for actual simulator validation. Asset loading,
FK agreement, queue timing, physical grasp success and learned-policy success
are separate gates. A successful mock rollout is not a grasp or model result.
