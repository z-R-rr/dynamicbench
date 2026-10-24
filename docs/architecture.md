# DynamicBench MVP architecture

Source audit baseline: RoboDojo `f57e1633ffed59761bc8e568cfe7530c4b33381c`.
LICENSE and the original README attribution are retained.

## Audited upstream path

`env_cfg/arx_x5.yml` selects dual_x5, 25 Hz observations and
`env_cfg/sim/sim_config.yml` (dt=.004, decimation=1).
`EvalEnv.take_action_batch` solves IK, calls `process_control_info`, then
consumes ControlManager synchronously in `step`. `process_control_info`
interpolates the first floor(80%) and holds the remaining controls.
`RobotManager.control_robot` uses Isaac Lab articulation joint position and
velocity targets; MetaControl also changes gripper targets from actual feedback.
`BaseEnv.sim_step` delegates to `CustomDirectRLEnv.sim_step`: scene write,
SimulationContext step, scene update, counters and interval events. Those
side effects must be retained.

`ObsManager` currently uses strict `collect_interval.is_integer()` and some
gripper observation fields come from previous commands, so DynamicBench reads
actual articulation state directly and validates the step ratio with tolerance.
Camera captures must be refreshed at the current physical state without stepping.

The conveyor task and config place a DynamicObject conveyor USD, photo, basket
and four objects. DynamicObject only wraps placement; motion must be checked
inside the USD and in PhysX. Task registration imports a module and class of
the same name. `_task.yml` supplies scene/robot overrides. The public README
explicitly describes an eval-only release; no official expert API is assumed.
Installation: `scripts/install.sh`; assets: `scripts/init_assets.sh` (HF
RoboDojo-Benchmark/RoboDojo, Assets subtree). X5 uses ARX.usd + X5A.urdf.

LIBERO-Dynamic reference: `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`.
Read runtime.py, control.py, sync_hold.py and rtc_queue.py: RTC skips the
elapsed prefix and replaces future queue; policy conditioning is separate.
Its sync_hold advances continuous substep latency and can idealize joint holds.
Here all modes remain on a 30 Hz grid; ceil quantization is explicit, and holds
use finite joint drives. This is a documented difference, not an exact port.

## Isolated implementation

CLI -> ConveyorCubeTask (BaseEnv subclass) -> original ARX articulation config
-> absolute EEF10 -> current-state seeded URDF IK -> bounded joint target
-> unchanged target for ten calls to BaseEnv.sim_step. The original
CustomDirectRLEnv owns every physics advance. No upstream evaluation or
ControlManager interpolation path is called or modified.

New code lives in dynamicbench/, scripts/dynamicbench/, docs/, results/.
The task is also exposed through a new task module without changing existing
task registration/default configs. Only .gitignore gets output exclusions.
Single X5 is simpler here because the standalone entry does not require dual_x5.
Its geometry and drive parameters come from the existing robot asset/config.
The layout resolver reads env_cfg/robot/dual_x5.yml (robot index 0) and
env_cfg/camera/camera_config.yml (cam_head) directly. Head lens intrinsics and
640x480 resolution come from the Gemini_345Lg template. Root quaternions are
normalized; the source files and their SHA-256 hashes are recorded in metadata.
No duplicate base position, head pose or head lens constants are maintained.
IK uses the asset URDF, checks FK against live link6, and avoids assuming the
server's older CuRobo installation matches the current v2 API.

The belt contact surface gets PhysX surface velocity (CPU physics), not cube
pose updates. The default is an explicitly identified collidable primitive.
The original conveyor USD has a native surface-velocity API and graph, so its
motion capability is not inferred from the DynamicObject wrapper. An optional
`--belt-source robodojo` adapter references that USD, disables its demo objects,
extra scene and render-tick graph inside the new instance, and owns its surface
speed on physics steps. Both belt sources passed live acceptance. The task cube
uses the exact single-environment path /World/envs/env_0/Cube: the old env_.*
regex also matched the original USD's nested demo Cube when it was deactivated,
causing Isaac Lab's deletion callback to unsubscribe the task cube's PLAY
initialization. See original_usd_fix.md for captured before/after traces.
Cube pose writes are reset-only.

Upstream setup_physics unconditionally forces GPU dynamics. Only this subclass
switches it back to CPU after the parent setup and checks the actual override.
The cube has sleep_threshold=0: a sleeping cube on a stationary kinematic belt
would ignore changes in contact surface velocity. It is never velocity-driven.
Contact sensors are created before physics startup. Rendering fences two camera
deliveries at the same frozen physical state; the physical counter is asserted
unchanged. This avoids the older SDK's first-frame idle-wait deadlock.

FixedRateController owns the integer physical clock. Scheduler never steps
physics. Fixed-delay chunks are computed from a snapshot and gated until their
available tick. This validates simulated inference latency only. No background model worker
is implemented. A future real model worker must receive copied observations,
avoid PhysX access, and timestamp result delivery. There is no real OpenWAM
experiment in this MVP.

The no-latency expert uses measured cube position/velocity and actual robot
state. Recorder receives observation[k], applied command[k], next observation
after ten steps, preserving N actions and N+1 observations. Privileged cube
state lives only under oracle_debug; model observations contain RGB and robot.
Success requires the same dynamic cube to remain lifted between both fingers.

## Validation boundary

Unit tests use instrumented fake backends to check timing/queues/schema; those
are never evidence of PhysX motion or grasp success. Integration commands must
save real states, RGB, videos and terminal outcomes. Every unavailable runtime
check is NOT RUN; unsuccessful physical attempts stay failures.


The 2026-10-24 live acceptance run passed belt transport at 0/.03/.06 m/s,
static grasp, three moving grasps at .03 m/s, and all three fixed-delay modes.
The strict reader revalidated all ten saved episodes, including final height
and bilateral contact evidence. See results/mvp_report.md for measured values,
source revisions and the remaining runtime regression limits.
On 2026-10-25, the original USD also passed all ten cases and strict readback;
30 CPU tests passed locally, on server and in CI.


## Live lift TCP and native fixtures

The default expert tracks current cube XY during lift and calibrates the held
cube offset relative to the actual finger midpoint at grasp establishment.
Each tick recomputes the EEF/TCP transform from measured finger positions;
finite-speed EEF10 commands and physics ownership are unchanged. This prevents
nominal-TCP feedback from chasing internal jaw motion. Raw nominal tracking and
fixed XY remain explicit ablations, with failed raw tracking episodes retained.

Native Room/Background/Light components load the original conveyor scene's
assets and authored lamp properties. Fixture collisions are disabled and the
old ground collider is retained but hidden, so the fixed baseline reproduces
its original physical trajectory under the new rendering. Policy inputs still
exclude all oracle_debug fields, including new pinch-frame diagnostics.
See lift_tracking_scene.md and results/lift_comparison.json for evidence.
