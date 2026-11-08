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
* RTC: same request/arrival gates, but drop the delay prefix and replace the
  future queue without client-side blending. Pass unconsumed original actions
  and delay steps to `predict_rtc` only when the adapter advertises support.
  Unsupported conditioning is recorded as false, never claimed as full RTC.
* One worker receives copied RGB (including wrist) and actual robot state only.
  It never receives the scene or privileged cube/contact data. Fixed/empirical
  latency can block wall time at the availability gate if inference is slow;
  that wait does not add another simulated delay. Measured latency is also
  supported, but depends on hardware contention and simulator throughput.

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
