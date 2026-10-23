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
IK uses the asset URDF, checks FK against live link6, and avoids assuming the
server's older CuRobo installation matches the current v2 API.

The belt contact surface gets PhysX surface velocity (CPU physics), not cube
pose updates. First audit the original conveyor USD; if its contact motion is
unreliable, use a clearly identified collision belt with the same contact-speed
mechanism and retain the audit evidence. Cube pose writes are reset-only.

FixedRateController owns the integer physical clock. Scheduler never steps
physics. Fixed-delay chunks are computed from a snapshot and gated until their
available tick. This validates simulated inference latency only. The optional
background policy adapter receives copied arrays and never accesses PhysX;
there is no real OpenWAM experiment in this MVP.

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
