# DynamicBench first-stage reproduction

Fork: https://github.com/z-R-rr/dynamicbench
Branch: feat/conveyor-rtc-expert-mvp. Original RoboDojo LICENSE/README retained.
Only the independent entrypoints below use the new timing and scene.

## Install

The tested server root is `/home/ubuntu/Workspace/dynamicbench` (no nested repo).
Use the existing Isaac environment, Python 3.11 and Isaac Sim 5.1.0. The report
pins the exact installed Isaac Lab package and checkout. For a fresh server,
follow upstream `bash scripts/install.sh -i` then `bash scripts/init_assets.sh`.
The upstream installer can update dependencies/submodules; the tested server
reuses its existing environment instead of modifying another project checkout.

```bash
git clone -b feat/conveyor-rtc-expert-mvp https://github.com/z-R-rr/dynamicbench.git /home/ubuntu/Workspace/dynamicbench
cd /home/ubuntu/Workspace/dynamicbench
git remote add upstream https://github.com/RoboDojo-Benchmark/RoboDojo.git
source /home/ubuntu/miniconda3/bin/activate RoboDojo
# Either download assets, or link the existing complete server asset tree:
ln -s /home/ubuntu/Workspace/RoboDojo/Assets Assets
# On a fresh machine without that tree, use instead:
# bash scripts/init_assets.sh
bash scripts/dynamicbench/prepare_server.sh
```

prepare_server installs small Python requirements and, when libGLU is absent,
extracts Ubuntu libglu1-mesa/libopengl0 packages under outputs/deps. It does not
need sudo. run_isaac.sh prepends that local library directory.

Required X5 files: Assets/Robots/x5/{ARX.usd,X5A.urdf}; original belt audit:
Assets/Object/RoboDojo/Dynamic/conveyor/{00000,00001}/object.usdz. Full asset
source is HF datasets/RoboDojo-Benchmark/RoboDojo, Assets subtree. Do not commit
assets/datasets/videos. `asset_audit.json` describes what was found.

## Run in PhysX

GPU 2 was selected for rendering after checking utilization. Check current GPU
usage before reproducing; CPU PhysX is deliberate for surface velocity support.
These commands are actual integration commands, not mocks. Use a fresh output
folder per run; existing episodes are never overwritten.

```bash
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test belt --ticks 100 --belt-speed .03 --seed 42 --output outputs/belt03
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test expert --belt-speed 0 --seed 42 --output outputs/static42
bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test expert --belt-speed .03 --seed 42 --output outputs/moving42
bash scripts/dynamicbench/run_isaac.sh collect_expert --headless --device cuda:2 --task pick_cube_from_conveyor --num-success 3 --max-attempts 10 --belt-speed .03 --seed 42 --save-failures --output datasets/conveyor_cube_v0
python scripts/dynamicbench/inspect_dataset.py datasets/conveyor_cube_v0
python scripts/dynamicbench/inspect_dataset.py datasets/conveyor_cube_v0/episode_seed42 --video outputs/preview42.mp4
for mode in no_latency sync_hold rtc_async; do
  bash scripts/dynamicbench/run_isaac.sh run_mvp --headless --device cuda:2 --test schedule --ticks 100 --belt-speed .03 --seed 42 --latency .25 --latency-mode "$mode" --output "outputs/schedule_$mode"
done
python -m pytest dynamicbench/tests -q
```

Expert collection is currently no_latency. The latency entry uses a deterministic
present-observation mock, H=32/stride=16. Unsupported delayed expert collection
is rejected explicitly; it is not a future-GT oracle. Fixed latency is simulated,
not measured GPU inference. Holds use finite drives. 250 ms -> ceil(7.5)=8 ticks
=266.667 ms, quantization +16.667 ms. RTC action i maps to request_tick+i;
returning discards elapsed prefix once. sync_hold reuses the entire stale chunk
after its wait, unlike LIBERO-Dynamic's sub-tick continuous hold latency.

## Inspect evidence

Each complete episode has trajectory.npz (all raw RGB/state/command frames),
metadata.json, trace.jsonl and head.mp4. `metrics.json` or collection_metrics.json
states actual attempts/successes/failures and source commit. Reader validates
N actions/N+1 observations, timestamps, real-vs-commanded fields and success
lift/contact evidence. Failed runs return a nonzero status and are never
labelled successful. See results/mvp_report.md for actual PASS/FAIL/NOT RUN.

Unit tests can run without Isaac Sim in a Python 3.11 virtualenv:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
pip install -r scripts/dynamicbench/requirements.txt
python -m pytest dynamicbench/tests -q
```
