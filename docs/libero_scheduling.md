# Conveyor evaluation with LIBERO-Dynamic queue semantics

Reference: LIBERO-Dynamic `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`,
`scripts/realtime_eval/control.py` and `strategies/{sync_hold,async_queue,rtc_queue}.py`.
This is a pinned protocol adaptation, not a claim of identical simulators.

Use `evaluate_policy` for this protocol. Historical `run_mvp --test schedule`
retains its old tick-quantized scheduler and old results remain reproducible.
Expert collection remains the uniform 30 Hz v1 format.

* Sync hold: infer only with an empty queue; advance latency through native
  physics while holding the previous joint targets, then execute action zero.
  No second queue delay. A 250 ms wait is 75 physics steps, not 8 control ticks.
  Nonintegral delays round upward to 1/300 s; requested and applied time are logged.
* Async queue: request when queue length <= threshold; fixed/empirical arrival
  is ceil(latency * 30) ticks. Drop only the number of queue actions actually
  executed since submission (underflow holds do not count). Replace future
  queue, blending overlap as 0.3 old + 0.7 new.
* RTC: same request/arrival gates, but drop the prefix through the actual
  release tick and replace the future queue without client-side blending.
  Nominal availability and actual release are recorded separately, so a late
  response cannot replay stale actions. Pass unconsumed original actions
  and delay steps to `predict_rtc` only when the adapter advertises support.
  Unsupported conditioning is recorded as false, never claimed as full RTC.
* One worker receives copied RGB (including wrist) and actual robot state only.
  It never receives the scene or privileged cube/contact data. Fixed/empirical
  latency can block wall time at the availability gate if inference is slow;
  that wait does not add another simulated delay.
* `--latency-source measured_wallclock` uses the worker's actual response-ready
  duration: policy call plus CPU output materialization/validation. The call-only
  duration is also logged as `policy_call_wall_seconds`. Adapters must return
  completed CPU actions; GPU submission alone is not a completed inference.
  While an async/RTC request is unfinished, each control boundary waits until
  its corresponding inference wall time or until the result completes. Fast
  simulation therefore cannot race ahead of the measured inference clock.
  Slow simulation continues queued actions until it reaches the calculated
  availability tick, without another wall-time sleep. Availability remains
  ceil(measured duration * 30): less than one control period of quantization.
  This aligns simulated latency; it does not guarantee real-time rendering.
  Worker startup, snapshot copying and external communication outside the
  policy call are not charged as model inference time. Fixed/empirical and
  synchronous/no-latency protocols do not use this asynchronous pacing.
  Requests log `clock_pacing_wall_seconds`, nominal `queue_delay_steps`, actual
  `effective_delay_steps`, and `release_lateness_steps`. RTC expires the actual
  age, whereas ordinary async still expires only executed queue actions.

Physical differences retained deliberately: finite joint drives instead of
idealized robot state freezing; 300 Hz PhysX instead of MuJoCo; absolute EEF10
instead of LIBERO delta actions. Weighted rotation columns are projected back
onto SO(3). Action horizon, control frequency, threshold and hold mode must all
be reported for comparisons.

Evaluation schema `dynamicbench.eef10.eval.v2` records every applied command or
hold segment with N+1 measured observations and physical timestamps. Its
`control_tick` is a recorded-transition index; trace `control_step` identifies
the scheduler tick, and `transition_kind=sync_wait` marks waiting segments.
Do not interpret v2 as a uniform training dataset. Videos are sampled at 30 Hz
from physical timestamps. Success still requires physical lift and bilateral
contact; a mock timeout is a diagnostic, not a successful grasp.

```bash
python scripts/dynamicbench/check_libero_conformance.py /path/to/pinned/LIBERO-Dynamic \
  --output results/libero_conformance.json
bash scripts/dynamicbench/run_isaac61.sh evaluate_policy --headless \
  --device cuda:3 --render-gpu 3 --belt-source robodojo \
  --validate-scheduling --ticks 64 --latency .25 --horizon 32 \
  --prefetch-threshold 16 --output outputs/libero_schedule_validation
```

A real model factory can be specified with `--policy-factory module:function
--policy-config provenance.json`. It must implement `predict(obs, tick, horizon)`
and return finite absolute world EEF10. The configuration must declare robot,
frame, rotation convention, gripper convention, frequency, checkpoint SHA-256,
normalization and training data. A compatible checkpoint is still required;
an adapter declaration alone is not a model experiment.
