# Dataset v1

Schema `dynamicbench.eef10.columns.v1`, NPZ (allow_pickle=False), JSON metadata,
trace JSONL and head.mp4 per atomic episode directory; manifest.jsonl references
only completed directories. One collector per output folder (file lock).

N transitions, N+1 observations. Row k is captured BEFORE command k at k/30;
row k+1 follows exactly ten physical steps. Tail frame is retained. `done` has
N entries with only the final entry true. No reset settle frames enter training.
`timestamp.sim` is episode-relative physical-step count / 300. Reset settling
150 physical steps is outside the episode and the absolute origin is checked.

- `observation.images.head`: uint8 [N+1,H,W,3], RGB, top-left origin.
- `observation.images.wrist`: same when available; metadata explicitly says
  not_available when absent. Never duplicate the head image as a wrist image.
- `observation.state.eef10`: [N+1,10], ACTUAL world link6 xyz in meters,
  rotation first TWO COLUMNS concatenated (R00,R10,R20,R01,R11,R21), measured
  gripper [-1,+1] from mean joint7/8 travel. World is right-handed, Z up.
- `action.eef10`: [N,10], absolute commanded world link6 pose, same convention.
  This is the wrist/link6 frame; finger pinch TCP is local [0.145,0,0] from the
  X5 gripper_bias, configurable and verified against physical grasp.
- `observation.state.qpos/qvel`: [N+1,8], actual joint1..joint8, radians/rad/s
  for six arm joints and meters/m/s for two prismatic fingers.
- `action.joint_target`: [N,8], actually submitted (possibly rate limited) drive
  target, constant for all ten physical steps. IK failures use previous legal
  target and are logged, never joint teleportation.
- `timestamp.sim`, `control_tick`, `physics_step`: [N+1]; exact control grid.
- `oracle_debug.cube_pose`: [N+1,7] xyz + wxyz; cube_velocity [N+1,3];
  finger_contact_forces [N+1,2], force magnitudes filtered to this cube.
  This namespace is privileged and excluded by the mock policy adapter.

Metadata contains task/seed/belt_speed, source commit, complete scene settings,
success/failure, frequency, frames and gripper convention. Successful episodes
also validate lifted cube bottom and bilateral contacts for success_hold_ticks.
Do not train on diagnostics/failures by default. Synthetic unit fixtures are
only tests; they are never recorded as physical demonstrations.

OpenWAM conversion: take RGB[k], actual EEF10[k]/qpos[k] as inputs and
command EEF10[k:k+H] as targets, mask episode padding, and exclude oracle_debug.
The Piper client's exact rotation convention has NOT been imported/verified;
convert rotation through a 3x3 matrix to the downstream client's convention
instead of assuming direct binary compatibility. Preserve action-vs-state,
frame, gripper sign, camera RGB order and timestamps during conversion.
