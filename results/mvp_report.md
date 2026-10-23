# DynamicBench MVP execution report — 2026-10-23 (Asia/Shanghai)

**Partial delivery. No physical grasp success or training demonstration is claimed.**
Actual code, tests, GitHub Fork, branch and Draft PR exist. The specified server
became inaccessible during the third PhysX startup attempt. The attempted belt
test has no verified completion/result; expert collection has not started.

## Repository and provenance

- Fork: https://github.com/z-R-rr/dynamicbench — GitHub API confirmed `fork=true`
  and parent `RoboDojo-Benchmark/RoboDojo`; not a copied standalone repository.
- Branch: `feat/conveyor-rtc-expert-mvp`.
- Upstream starting SHA: `f57e1633ffed59761bc8e568cfe7530c4b33381c`.
- Implemented/tested code SHA: `9cfd801446cf9ae7537b194d71c449f7503414c5`.
- Draft PR to this Fork's **main**, not upstream:
  https://github.com/z-R-rr/dynamicbench/pull/1
- Local root: `/Users/riri/code/dynamicbench`.
- Server root: `/home/ubuntu/Workspace/dynamicbench` (no nested repository).
- Last server checkout: `c320fea637ae44c53db16ad217b72f8ae1774180`.
  Later local commits are pushed but not yet synchronized to the disconnected server.
- LIBERO-Dynamic reference: `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`.
  Read its runtime/control and sync_hold/rtc_queue source; no MuJoCo code copied.
- Original LICENSE and attribution retained. No force push, deletion of user
  data, modification of OpenWAM or modification of the existing RoboDojo checkout.

The final report itself is a subsequent documentation commit. Use `git rev-parse
HEAD` for that delivery revision; source/data metadata keeps the code revision
actually used for each run, not an invented self-referential report hash.

## Implementation

`dynamicbench/envs/` adds integer clock, absolute EEF10 rotation validation,
current-state seeded bounded URDF IK, a BaseEnv-derived ARX X5 scene and a
FixedRateController. Each tick observes first, updates a finite joint drive once
and calls the unchanged native `sim_step()` ten times, with the same joint target.
It does not invoke `take_action_batch`, `process_control_info`, TOPP, cube attach,
per-frame cube pose writes, or joint state teleportation outside reset.
Cube bottom height and actual cube-filtered bilateral finger contacts must hold
for 15 control ticks to satisfy success. That criterion is implemented but has
NOT been demonstrated in physical execution.

`dynamicbench/experts/` contains a closed-loop privileged motion/track/descend/
close/lift expert. `dynamicbench/runtime/` contains fixed-delay queue modes and a
present-observation mock (H=32, stride=16). Chunk compute is immediate Python
computation with simulated availability gating: this is not real GPU asynchronous
inference. A 250 ms delay becomes 8 ticks = 266.667 ms (+16.667 ms). sync_hold
holds finite drives and uses the full stale chunk after its wait; this differs
from LIBERO-Dynamic's continuous sub-control-tick sync latency.

`dynamicbench/data/` and shared `episode.py` provide atomic NPZ/RGB/JSON/MP4
recording, N actions/N+1 observations, applied vs actual state, strict validation,
source/settings metadata and privileged debug separation. `scripts/dynamicbench/`
contains run/collect/inspect and scoped dependency setup. The task module is an
independent CLI task entry; legacy EvalEnv invocation is deliberately unsupported.
`docs/` defines architecture/schema/commands, and a new GitHub workflow runs CPU
contracts. No original task/config/evaluation file changed; .gitignore only adds
output exclusions and an exception for the new data package.

The original conveyor USDs are accessible and statically scanned for motion
attributes. This delivery selects a collidable primitive belt with PhysX surface
velocity. **The original USD's actual motion and the replacement's actual cube
transport are both unvalidated**, so the requested original-belt check/fallback
justification remains incomplete. Config accepts 0/.03/.06 m/s, but currently no
speed is listed as physically validated. No successful rate is assumed.

EEF10 is world link6 xyz + concatenated first two rotation columns + normalized
measured/commanded gripper (-1 closed, +1 open). It is explicitly versioned; exact
Piper/OpenWAM rotation compatibility was not assumed. Head and independent wrist
RGB sensors are implemented; real image/video alignment still requires the
native integration run. Synthetic CPU fixtures are only unit tests.

## Environment and executed operations

Authenticated as `z-R-rr`; checked target absence, forked using
`gh repo fork RoboDojo-Benchmark/RoboDojo --fork-name dynamicbench --clone=false`,
then cloned and configured origin/upstream and the feature branch. Commits were
pushed before server fetch/fast-forward. Server fetch required the already
available proxy: `git -c http.proxy=http://127.0.0.1:7897 fetch origin`.

Server: `vm-zhangR`, Ubuntu 22.04.5, NVIDIA driver 580.95.05, four RTX 4090s.
Existing environment `/home/ubuntu/miniconda3/envs/RoboDojo`:
Python 3.11.16, Isaac Sim package 5.1.0.0, Isaac Lab package 0.54.3,
experience isaaclab.python-2.3.2, Isaac Lab checkout afca7b09, Torch
2.7.0+cu128, NumPy 1.26.0. The existing CuRobo checkout is
895c6517243f8cb091c73c018c8167192d39599a; the new executor does not require its
version-specific planner API. Renderer GPU 2 was chosen after inspecting usage.
CPU PhysX is used for the surface-velocity scene. Other GPU jobs were not stopped.

Assets symlink in the new repository:
`Assets -> /home/ubuntu/Workspace/RoboDojo/Assets` (existing 39 GB tree).
Required robot files ARX.usd and X5A.urdf were present. Original conveyor assets:
`Assets/Object/RoboDojo/Dynamic/conveyor/{00000,00001}/object.usdz`.
Fresh asset download: `bash scripts/init_assets.sh` (HF dataset
RoboDojo-Benchmark/RoboDojo, Assets subtree).

- Attempt 1, source 46b83c1: missing `libGLU.so.1` broke MDL/RTX shader startup.
- Extracted Ubuntu `libglu1-mesa` and `libopengl0` with `apt-get download` and
  `dpkg-deb -x` into `outputs/deps/root`; no system package/restart changes.
  Existing imageio/imageio-ffmpeg dependencies were checked with pip.
- Attempt 2, source 0be0534: Simulation App and physical ARX scene initialized;
  late-created contact sensors lacked `_timestamp`. Corrected by creating them
  as InteractiveScene sensors before startup; also added independent wrist RGB.
- Attempts 1/2 left Kit cleanup processes stuck. Only our exact test PIDs 19213
  and 20545 were terminated after checking their commands. No unrelated job killed.
- Attempt 3, source c320fea: actual 300 Hz config and physical scene creation
  logged; last observed log included startup/asset/fabric initialization.
  No final metrics/episode result was retrieved before network failure.
- ZeroTier TCP22 then timed out/no route. Fallback 192.168.3.35:31026 has the
  same previously trusted ED25519 fingerprint, but accepted TCP then closed SSH
  without a usable handshake. No network service restart/reset was attempted.

Verified remote log files:

```text
/home/ubuntu/Workspace/dynamicbench/outputs/belt03/run.log
/home/ubuntu/Workspace/dynamicbench/outputs/belt03_v2/run.log
/home/ubuntu/Workspace/dynamicbench/outputs/belt03_v3/run.log
```

The last belt-test PID was 21694; **its current state is unknown after connection
loss**. Check that exact command/PID after reconnecting before starting another
Kit process. The repository is intentionally kept in place.

## Test evidence

| Acceptance check | Status | Observed evidence |
| --- | --- | --- |
| Clock/ZOH/queues/schema/failure contracts | PASS | 26 local Python 3.11 tests |
| Same CPU contracts on server | PASS | Earlier 22-test revision c320fea |
| GitHub CPU CI | PASS | [run 39976510653](https://github.com/z-R-rr/dynamicbench/actions/runs/39976510653) at 9cfd801 |
| Ruff, compilation, git diff whitespace | PASS | Local checks executed |
| Original environment/config/eval/LICENSE/README unchanged | PASS | Exact git source diff against upstream SHA |
| Belt displacement after 100 physical control ticks | NOT RUN | Launched; completed result unverified due network loss |
| Static real grasp + video/raw dataset | NOT RUN | Expert run not started |
| .03 m/s real moving grasp | NOT RUN | Expert run not started |
| Three successful expert demonstrations | NOT RUN | Collection not started |
| Three latency modes in live PhysX | NOT RUN | Mock queue CPU tests only |
| Old-task GPU/runtime regression | NOT RUN | Source preservation checked, runtime not executed |
| Original conveyor live motion | NOT RUN | USD presence/static scan only |

Grasp attempts **0**, successes **0**, success rate **undefined**. Three simulator
startup attempts are not three grasp attempts. No successful episode, video or
manifest exists. `sample_episode_manifest.json` explicitly contains no samples.
No model weights/training were used, and measured real GPU inference latency is
not available. Mock Python computation is timed separately in request metadata
only when an episode can actually run.

## Reproduce after connection recovery

All full install/asset/run/collect/inspect/test commands are in
[quickstart_dynamicbench.md](../docs/quickstart_dynamicbench.md). Continue in this
order, retaining existing directories and checking status before synchronization:

```bash
cd /home/ubuntu/Workspace/dynamicbench
git status --short
git branch --show-current
git rev-parse HEAD
git -c http.proxy=http://127.0.0.1:7897 fetch origin
git merge --ff-only origin/feat/conveyor-rtc-expert-mvp
source /home/ubuntu/miniconda3/bin/activate RoboDojo
bash scripts/dynamicbench/prepare_server.sh
python -m pytest dynamicbench/tests -q
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test belt --ticks 100 --belt-speed .03 --seed 42 --output outputs/belt03_recovery
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test expert --belt-speed 0 --seed 42 --output outputs/static42
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test expert --belt-speed .03 --seed 42 --output outputs/moving42
bash scripts/dynamicbench/run_isaac.sh collect_expert --headless --device cuda:2 --num-success 3 --max-attempts 10 --belt-speed .03 --seed 42 --save-failures --output datasets/conveyor_cube_v0
python scripts/dynamicbench/inspect_dataset.py datasets/conveyor_cube_v0
for mode in no_latency sync_hold rtc_async; do
  bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test schedule --ticks 100 --belt-speed .03 --seed 42 --latency .25 --latency-mode "$mode" --output "outputs/schedule_$mode"
done
```

First retrieve the third startup traceback/result; if startup completes, verify
actual belt displacement before attempting a grasp. If contacts/IK fail, inspect
live joint limits, URDF-vs-USD FK, pinch geometry and cube-filtered forces using
recorded state. Tune only from those physical measurements. Validate original
conveyor motion before claiming the primitive replacement is required. Do not
relax lift/contact criteria, tag failed episodes as success, or report unit
fixtures as integration evidence. Full intended acceptance remains unfinished.
