# Conveyor scheduling and embodiment validation — 2026-11-08

The independent evaluator now follows pinned LIBERO-Dynamic queue/time
semantics on the existing physical conveyor task. Piper and Franka adapters
were added alongside the original X5. No upstream RoboDojo environment,
default robot configuration, task configuration, or submodule pin was changed.

Protocol reference: `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`.
Runtime source: `6d4eac23fbbc67b223f9ced547fc92842118178e`.
Host: `vm-zhangR`, Isaac Sim `6.1.0.0`, Isaac Lab 3 EA, CPU PhysX and GPU 3 RTX.
GPU 3 was the least-used GPU when the accepted runs were started.

## Evidence

* 57 CPU tests passed, including partial physical-step waits, strict v1/v2
  timestamps, fixed-joint FK, worker snapshot isolation, RTC conditioning
  arguments, underflow and late-prefix handling.
* Differential execution against the actual pinned reference checkout passed
  9 cases / 1,080 decisions: three modes at 0, 250 ms and 1.2 s latency.
  Queue lengths, underflow flags and non-underflow executable actions agree. See
  [libero_conformance_20261108.json](libero_conformance_20261108.json).
* Public HEAD `fd45961603cc50debaa959be52116626bc0fda1f` was checked through
  GitHub: the runtime, control module and all three strategy files have identical
  Git blob hashes to the pinned reference. See
  [libero_reference_audit_20261108.json](libero_reference_audit_20261108.json).
* GitHub Actions including the model-frame contract guard (`aedfe79`) passed
  on Python 3.11 and 3.12:
  [run 42457241846](https://github.com/z-R-rr/dynamicbench/actions/runs/42457241846).

Physical expert grasps, seed 42, original conveyor USD, belt speed 0.03 m/s:

| Embodiment | Outcome | Control ticks | Peak EEF XY motion after contact | Peak cube XY motion after contact |
|---|---|---:|---:|---:|
| X5 | PASS | 82 | 0.342 mm | 7.732 mm |
| Franka Panda | PASS | 104 | 2.664 mm | 3.640 mm |
| Piper | PASS | 105 | 0.953 mm | 1.143 mm |

These are one successful physical episode per embodiment, not a multi-seed
robustness estimate. Original height, bilateral-contact, hold-duration and
post-contact motion gates were retained. All three episodes passed strict
NPZ readback; head/wrist frames were visually reviewed, and effective render
settings passed the runtime audit. Each exact source revision and trajectory
hash is in [embodiment_grasps_20261108.json](embodiment_grasps_20261108.json).

The first X5 scheduling run executed 64 actions per mode at 250 ms latency:

| Mode | Physics steps | Physical duration | Cube X displacement | Theory |
|---|---:|---:|---:|---:|
| No latency | 640 | 2.1333 s | 6.3984 cm | 6.4000 cm |
| Sync hold | 790 | 2.6333 s | 7.8984 cm | 7.9000 cm |
| Async queue | 640 | 2.1333 s | 6.3984 cm | 6.4000 cm |
| RTC queue | 640 | 2.1333 s | 6.3984 cm | 6.4000 cm |

Sync applies two waits of 75 physics steps, without charging another queue
delay. Async/RTC each execute 24 old queued actions while later requests are
pending. Async preserves the first incoming prefix during initial underflow;
RTC drops eight actions. Full traces and source metadata are summarized in
[libero_schedule_sim_20261108.json](libero_schedule_sim_20261108.json).

## Reproduction and artifacts

Repository: `/home/ubuntu/Workspace/dynamicbench` on the server. Use a new output
directory for every run; accepted and diagnostic episodes are never overwritten.

```bash
bash scripts/dynamicbench/run_isaac61.sh evaluate_policy --headless \
  --device cuda:3 --render-gpu 3 --embodiment x5 --belt-source robodojo \
  --validate-scheduling --ticks 64 --latency .25 --horizon 32 \
  --prefetch-threshold 16 --output outputs/x5_schedule_new \
  --kit_args="--/plugins/carb.tasking.plugin/threadCount=8 --/crashreporter/preserveDump=true"
bash scripts/dynamicbench/run_isaac61.sh run_mvp --headless \
  --device cuda:3 --render-gpu 3 --embodiment piper --belt-source robodojo \
  --test expert --seed 42 --output outputs/piper_grasp_new \
  --kit_args="--/plugins/carb.tasking.plugin/threadCount=8 --/crashreporter/preserveDump=true"
```

Use `franka` for the Panda adapter. Piper defaults to the sibling RoboTwin
asset directory; override it with `--piper-asset-root` if necessary.
See [protocol details](../docs/libero_scheduling.md) and
[embodiment configuration](../docs/embodiments.md).

Accepted expert directories:

* `outputs/official_config_smoke_20261108` (X5, source `f0ae4f1`)
* `outputs/franka_expert_drives_20261108` (Franka, source `24e1e94`)
* `outputs/piper_native_root_20261108` (Piper, source `6d4eac2`)

Final protocol runs use `outputs/{x5,franka,piper}_schedule_final_20261108`.
All 12 final cases passed on source `6d4eac2`. Independent readback verified
NPZ schemas, every trace/physics timestamp, constant joint targets during sync
waits, 64 scheduler actions per case, 24 old-queue executions during asynchronous
inference, prefix expiration and decoded video frame counts. Results and hashes:
[scheduling_final_audit_20261108.json](scheduling_final_audit_20261108.json).
Local expert previews and full expert episodes are under the chat workspace's
`outputs/scheduling_20261108/{x5,franka,piper}_expert` directories.

## Diagnosed failures and boundaries

The first Franka diagnostic with source damping tracked too slowly; new task
adapters use explicitly recorded finite arm gains (2000 stiffness / 40 damping),
without changing source files or X5 gains. Piper's native 6.1 import required
resolving nested rigid-body paths and retaining its authored world fixed joint;
re-fixing the referenced articulation caused a Newton-schema relocation error.
GPU 2 startup stalled waiting for RTX shader compilation; those owned diagnostic
processes were stopped and the accepted jobs ran on GPU 3. Diagnostic folders
are retained and are not training demonstrations.

Physics remains finite-drive PhysX, not idealized joint-state freezing.
Synchronous delays round to physical steps (<=3.333 ms resolution), while
async/RTC arrivals stay on the 30 Hz control grid. EEF10 rotation blending
requires projection back to SO(3), unlike Euclidean LIBERO delta actions.
Franka/Piper wrist mounts and task placements are explicitly task-defined;
only X5 uses the official X5 camera mount.

No compatible learned checkpoint was loaded or evaluated. The scheduling runs
use a copied-observation mock, so their expected task timeout is not a grasp
failure or a learned-policy success rate. RTC queue semantics are validated;
model-side RTC conditioning has only contract tests, not a learned-model run.
