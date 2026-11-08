# Conveyor task embodiments

Select `--embodiment x5|franka|piper` on `run_mvp`, `collect_expert`, or
`evaluate_policy`. The default remains X5. Belt, cube, room, lights and head
camera stay the same. All embodiments inherit the front installation pose from
the layout resolver; robot-specific joints and pregrasp IK change.

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

Franka and Piper inherit the same front mounting slot as X5, resolved from
RoboDojo `env_cfg/robot/dual_x5.yml` (robot index 0): position
[-0.3,-0.45,0.765], WXYZ quaternion [sqrt(0.5),0,0,sqrt(0.5)]. There is no
separate hardcoded root for the new adapters. `installation_pose_source` records
the source config. This reuses the X5 installation slot; it is not an official
Piper/Franka conveyor configuration.

Pregrasp joints are solved deterministically from each URDF using the inherited
root rotation, after the selected production conveyor geometry is resolved.
Changing compact-only dimensions cannot alter the official-profile pregrasp.
The grasp orientation also rotates with that mounting frame.
Franka approaches vertically. Piper retains its 30-degree inclined approach
because of its joint5 limit; the jaw yaw avoids the joint6 wrap seam.

New-adapter arm drives use task-specific stiffness 2000 and damping 40, with
the source effort limits retained. The first Franka diagnostic using its
original high damping could not catch the moving target promptly. All overrides
are recorded in `settings.robot`; X5 gains are unchanged. Finger effort remains
15 N for Franka and is capped at the Piper URDF's 10 N.

X5 keeps its official asset camera mount. Franka and Piper use an explicitly
task-mounted D435 at the declared EEF, local position [.05,0,.025], WXYZ
[0,1,0,0]. This is not presented as an official wrist-camera calibration.
The front-layout change preserves these local mounts and the D435 lens. Wrist
world poses naturally follow the robot; the head-camera world pose, conveyor,
cube, room, lights and renderer settings are unchanged by this change.

Piper assets are discovered in the sibling RoboTwin checkout or supplied via
`--piper-asset-root /absolute/path/to/piper`. Conversion uses Isaac Lab's native
URDF importer into `outputs/assets/piper/<urdf-hash>/piper.usd`; source assets
are not edited. Fixed links are retained; both finger joints are driven.
Generated USD and large assets are not committed.

The 6.1 importer authors a nested `Geometry/...` link hierarchy and a world
fixed joint. The adapter resolves rigid-body paths from the generated USD and
retains that fixed joint. Reapplying `fix_root_link=True` tries to relocate a
referenced Newton schema in this Lab version and fails; the adapter avoids
that redundant operation without modifying Isaac Lab or source assets.

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
