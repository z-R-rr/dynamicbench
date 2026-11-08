# Measured wall-clock latency: conveyor validation

Validated source: `40015786826240d74761a855d1f83fcb0f78f815`.
The accompanying [machine-readable report](measured_clock_validation_20261108.json)
contains request timing, schema checks, runtime versions, artifact hashes and
the separate expert regression. Later documentation-only commits do not change
the source attribution of these runs.

The official conveyor/room/X5 scene ran under Isaac Sim 6.1, using CPU PhysX at
300 Hz and control at 30 Hz, seed 42, belt speed 0.03 m/s, horizon 32 and prefetch
threshold 16. Each scheduling case applied 64 control intervals. A diagnostic
mock slept for 250 ms inside its worker; the scheduler measured the actual call
and CPU output conversion/validation. No trained model or checkpoint was used.

| Case | Physics steps | Rollout wall time | Released request delays | First dropped prefix | Audit |
| --- | ---: | ---: | --- | ---: | --- |
| no_latency | 640 | 14.87 s | 0 | 0 | PASS |
| sync_hold | 794 | 15.56 s | 77 physics steps per request | 0 | PASS |
| async_queue | 640 | 12.20 s | 8 control ticks | 0 | PASS |
| rtc | 640 | 12.07 s | 8 control ticks | 8 | PASS |
| async_queue, slowed simulator | 640 | 15.36 s | 8 control ticks | 0 | PASS |
| rtc, slowed simulator | 640 | 15.69 s | 8 control ticks | 8 | PASS |

The slowed cases add 50 ms of wall time per ten physics steps without changing
the physics timestep. Their request ticks, release ticks, discarded prefixes,
enqueued lengths and overlap counts exactly match the corresponding normal
cases. Async/RTC response-ready durations were 252.216–259.784 ms, quantizing to
8 control ticks. All released requests had zero release lateness. Physics steps
began while inference was active (78/78 normal async/RTC; 40/44 slowed cases).
Old queued actions continued during pending requests (16 async, 24 RTC).

Sync retains the existing infer-then-replay semantics: hold finite joint targets
and advance physics for the measured delay before applying the returned action.
Its two measured waits were about 255.7 ms each, hence 77 physics steps each,
and 640 + 154 = 794 total. It does not claim concurrent wall-time simulation.
The conveyor displacement error was below 0.04 mm in every diagnostic case.
All seven saved episodes passed the strict trajectory reader, including head
and wrist RGB, timestamps and N+1 observations. Mock rollouts intentionally end
at their diagnostic timeout; this is not a grasp success metric.

The separate no-latency expert completed a physical grasp/lift in 112 control
steps (3.733 s). Contact motion checks passed: peak post-contact EEF XY excursion
0.468 mm, object XY excursion 7.553 mm; lift-phase world X drift 0.561 mm.
This is one regression seed, not a policy success-rate estimate.

CPU verification: 89 tests passed, including fast/slow simulated clocks, output
materialization timing, old-queue continuation and delayed RTC expiration.
Differential checks against LIBERO-Dynamic `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`
passed all 9 cases / 1080 control decisions. Lint and whitespace checks passed.
The GPU simulator itself ran slower than real time; the fast-simulator pacing
branch is covered by deterministic CPU tests, not claimed as a GPU measurement.

Reproduce after selecting the GPU with the least used memory:

```bash
bash scripts/dynamicbench/run_isaac61.sh validate_measured_clock \
  --headless --device cuda:3 --render-gpu 3 --embodiment x5 \
  --belt-source robodojo --ticks 64 --latency .25 \
  --output outputs/measured_clock_acceptance
```

The diagnostic entry always uses `measured_wallclock`; `--latency` controls only
its worker sleep. General policy evaluation still requires explicit
`--latency-source measured_wallclock`. This correction covers worker invocation
through usable CPU actions. Observation capture/copy, worker startup and
external transport outside the adapter call remain outside the measured scope;
this is not a complete observation-to-actuation latency implementation.
