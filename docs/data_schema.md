# Dataset v1

This document describes uniform expert/MVP data. The new `evaluate_policy`
entrypoint writes `dynamicbench.eef10.eval.v2`, documented in
[libero_scheduling.md](libero_scheduling.md), with explicit physical timestamps
and separate synchronous-wait transitions. It must not be interpreted as a
uniform training trajectory. Piper/Franka EEF links and joint dimensions are
declared in metadata; see [embodiments.md](embodiments.md).

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
- `observation.state.eef10`: [N+1,10], ACTUAL world EEF xyz in meters
  (`link6` for X5/Piper, `panda_hand` for Franka; explicit in metadata),
  rotation first TWO COLUMNS concatenated (R00,R10,R20,R01,R11,R21), measured
  gripper [-1,+1] from mean joint7/8 travel. World is right-handed, Z up.
- `action.eef10`: [N,10], absolute commanded world EEF pose, same convention.
  This is the wrist/link6 frame. Approach uses nominal local TCP [0.145,0,0];
  the contact_anchor default latches world link6 XY/orientation and ramps Z.
  Legacy track_cube can use the measured finger midpoint and grasp offset.
  Internal control references never change the action frame or EEF10 schema.
- `observation.state.qpos/qvel`: [N+1,D], actual articulation joint order;
  D=8 for X5/Piper and D=9 for Franka. Arm units are radians/rad/s;
  the two prismatic fingers use meters/m/s.
- `action.joint_target`: [N,D], actually submitted (possibly rate limited) drive
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


Optional privileged diagnostics (all N+1 by 3, meters except velocity):
- oracle_debug.cube_in_eef: cube relative to actual link6 axes/origin.
- oracle_debug.finger_midpoint_in_eef: actual finger-body midpoint in link6 axes.
- oracle_debug.cube_in_pinch_frame: cube relative to that midpoint, with link6 axes.
- oracle_debug.belt_surface_velocity: runtime API value in world m/s.

EEF-relative displacement includes internal jaw-center motion. Use pinch-relative
changes to assess translation within the held grasp; do not equate world motion
or total path length with slip. Metadata records lift_xy_mode, lift_tcp_frame,
and scene.source. Old datasets remain readable without these additive fields.

Metadata also records contact_motion from the first finger contact through the
terminal observation, including the pre-lift interval. contact_anchor requires
contact_motion_passed as well as the existing height/hold success. These metrics
do not change raw array indexing or the action coordinate convention.
