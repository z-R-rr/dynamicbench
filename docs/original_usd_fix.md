# 原装传送带 USD 初始化修复（2026-10-25）

## 已实测定位的根因

导入原装 USD 后，适配器会禁用它自带的演示方块：

```text
/World/envs/env_0/OriginalConveyor/Cube
```

本任务自己的方块此前使用 `{ENV_REGEX_NS}/Cube`，展开为 `/World/envs/env_.*/Cube`。
服务器固定版本 Isaac Lab 的 `AssetBase._on_prim_deletion` 用 Python `re.match` 检查被删除/禁用的 prim 是否属于该资产。
其中 `.*` 可以跨越 `/`，因而把原装演示方块误认为本任务方块，并调用 `_clear_callbacks()`，取消本任务方块的 PLAY 初始化订阅。

临时追踪实际捕获了以下顺序：

```text
callbacks removed for /World/envs/env_.*/Cube by /World/envs/env_0/OriginalConveyor/Cube
before play cube: initialized=False subscribed=False
```

随后的 `sim.reset()` 没有该方块的初始化回调可执行，`RigidObject._initialize_impl()` 没有创建 `_data`。
`DirectRLEnv.__init__ -> scene.update -> rigid_object.update -> self._data.update` 随后触发 AttributeError。
这也解释了为什么当时保存的初始化回调异常为空：该回调根本没有执行。

这次初始化错误与此前独立遇到的 GPU0 显存不足是两条不同的故障链；不能把它归因于 USD 损坏。

## 最小修复

本任务明确固定 `num_envs=1`，将方块路径限定到精确的单环境路径：

```python
# Before
prim_path = "{ENV_REGEX_NS}/Cube"

# After (resolved value of CUBE_PRIM_PATH)
prim_path = "/World/envs/env_0/Cube"
```

`dynamicbench/envs/scene_paths.py` 定义唯一的 `CUBE_PRIM_PATH`，方块和左右接触过滤器共同使用它。
本任务方块仍由标准 Isaac Lab RigidObject 在 PLAY 事件初始化，保留原生物理句柄、状态缓冲和 scene.update 流程。
修复不修改 Isaac Lab、RoboDojo 的旧任务或源 USD。临时追踪子类已移除，正式代码继续使用原生 RigidObject。
未来扩展多环境时，需要相应设计逐环境的精确路径/匹配规则；当前修复专用于本期单环境契约。

修复后同一探针捕获：

```text
before play cube: initialized=False subscribed=True
initialize /World/envs/env_0/Cube
initialized /World/envs/env_0/Cube: data=True
```

## 已执行验证

- 回归测试重现旧表达式误匹配，并验证精确路径不受外来演示方块删除影响，仍正确响应本方块/本环境的删除。
- 本地与服务器 30 项 CPU 测试通过。
- 原装 USD，速度 .03 m/s，100 控制 tick / 1000 物理步：位移 0.0999726504 m，期望 0.1 m，通过。
- 正式代码原装 USD 验收 10/10 通过：三档输送、静态抓取、.03 动态抓取 3/3、三个 RTC 模式；全部 episode 和视频帧数通过严格回读。
- 运行代码 SHA：`1a0163ae0f60848849fa2eb153d0c83d6abb9951`。
- 完整原装 USD 验收结果见 `results/physical_acceptance_original_usd.json`（各次实验保留真实 source_commit）。

独立复现（先按当前 GPU 使用情况选择设备；下例为本次实际使用的 GPU3）：

```bash
cd /home/ubuntu/Workspace/dynamicbench
source /home/ubuntu/miniconda3/bin/activate RoboDojo
unset CUDA_VISIBLE_DEVICES
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
bash scripts/dynamicbench/run_isaac.sh validate_mvp --headless --device cuda:3 --belt-source robodojo --num-success 3 --max-attempts 10 --seed 42 --output outputs/acceptance_original_usd_repro --kit_args="--/validate/iommu/enabled=false --/validate/p2p/enabled=false --/plugins/carb.tasking.plugin/threadCount=8"
python scripts/dynamicbench/inspect_dataset.py outputs/acceptance_original_usd_repro
```

默认配置保留 primitive；`--belt-source robodojo` 显式选择原装传送带。既有 primitive 数据仍保留原场景、原 source_commit，不重新标注成 USD 数据。
