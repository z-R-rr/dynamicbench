# 官方布局生产接入与独立验证（2026-11-08）

默认生产任务已读取官方保存的传送带变换、X5 初始关节、双臂布局和可见 Ground。
`run_mvp`、`collect_expert`、`evaluate_policy` 共用同一配置入口；历史小传送带通过
`--layout-profile compact` 显式复现。实现说明见 [production_layout.md](production_layout.md)。

本次服务器运行源提交为 `183866278f3e7eecda9c22e119293a4f9b383fec`，
分支为 `feat/official-conveyor-layout`。运行在独立目录
`/home/ubuntu/Workspace/dynamicbench-official-layout-20261108`，使用 Isaac Sim 6.1.0.0、
CPU PhysX（300 Hz）、GPU 3 RTX 渲染及 30 Hz 控制。没有修改主聊天的工作目录或运行任务。
GPU 2 的首次驱动着色器编译未进入任务即停止，日志保留，不计入物理验收。

## 布局和画面

- 实际 USD 根变换与官方保存布局逐项一致：scale `[1,1,1]`，位置
  `[-2.08,0.175,0.05]`，WXYZ `[0,0,0,1]`。
- 实测 Belt 世界边界为 `[-2.078043397,-0.274998143,0.746597077]` 至
  `[1.849253480,0.625001237,0.762211945]`；重置后方块底面与该带面顶部相差约
  `0.00000034 m`，同时用实际落稳位置核对了支撑高度。
- 主相机 `[0,-0.41,1.308]`；左 X5 底座 `[-0.3,-0.45,0.765]`；
  六关节实际初始值全部为零。官方右 X5 已实例化并保持有限驱动，任务仍仅控制左臂。
- Ground 使用原始 mesh/UV、变换顺序、MDL 和物理材质；Room 原有碰撞属性保持一致。
- quality、DLAA、反射、GI 等 18 项运行时设置核对通过。6.1 与旧引擎逐像素相同不在验收范围内。
- 实际头部/腕部帧已人工查看：带面占据主体视野，双臂、相机支架和深色 Ground 正常出现。

这是当前单方块任务的布局对齐；未加入原图像识别任务的篮子、展示板或额外目标物。

## 抓取和采集

带速 `0.03 m/s`，种子 42、43、44，三个尝试全部成功。另有一个独立 seed 42 smoke 成功，
不把它当成第四个独立种子。

| 种子 | 控制步 | 时长（s） | lift 高度增量（cm） | lift X 净位移（mm） | lift 最小单指力（N） |
| --- | ---: | ---: | ---: | ---: | ---: |
| 42 | 112 | 3.7333 | 11.157 | -0.562 | 14.968 |
| 43 | 113 | 3.7667 | 11.191 | 0.135 | 14.964 |
| 44 | 112 | 3.7333 | 11.172 | 0.427 | 14.930 |

三个种子接触后的水平目标变化均为 0，实际 EEF 水平最大偏移不超过 0.469 mm；
接触至抬升前方块仍有约 5 mm 位移，未把它隐去。持续双指接触及抬升保持条件均通过。
全过程保留 `0.03 m/s` 传送带表面速度，没有为验收关闭传送带或直接写方块状态。
新示教使用独立目录保存，未覆盖旧布局示教。N+1 观测、30/300 Hz 时间戳、双指成功证据和视频帧数均严格读回通过。

## 调度

固定延迟 250 ms，chunk horizon 32，prefetch threshold 16，64 个控制动作；
四个模式均在相同种子和官方布局重置后运行。

| 模式 | 物理步 | 仿真时长（s） | 同步等待物理步 | 方块 X 位移（cm） | 初始队列欠载步 |
| --- | ---: | ---: | ---: | ---: | ---: |
| no_latency | 640 | 2.1333 | 0 | 6.3984 | 0 |
| sync_hold | 790 | 2.6333 | 150 | 7.8984 | 0 |
| async_queue | 640 | 2.1333 | 0 | 6.3984 | 8 |
| rtc | 640 | 2.1333 | 0 | 6.3984 | 8 |

两次 sync 等待各 75 个物理步，末尾各含一个 5 步区间；等待期间传送带继续运动。
`async_queue` 第一次丢弃前缀为 0，后续已释放请求丢弃 8、重叠 8；
RTC 的已释放请求均丢弃 8、无重叠混合。记录时间线和视频重采样帧数全部通过。
这是确定性 mock 的调度验证，`rtc_conditioning_used=false`；没有声称真实模型、RTC 条件推理或延迟抓取成功。
Piper/Franka 的前方布局由主聊天继续开发，本报告不将它们旧布局的成功算入此次验收。

## 证据和复现

- [完整指标、源文件哈希、45 个产物的 SHA-256 清单](../results/official_layout_production_20261108.json)。
- 服务器目录内：`outputs/official_smoke02`、`outputs/official_collection01`、`outputs/official_scheduling01`。
- 本地完整产物：`/Users/riri/Documents/Codex/2026-10-23/files-mentioned-by-the-user-dynamicbench/outputs/official_layout_production_20261108`。
- 实际画面：该目录内 `smoke02/official_grasp_contact_sheet.png` 和 `smoke02/official_head_initial.png`。
- 复现命令见 [production_layout.md](production_layout.md)，本次使用 GPU 3；重新运行时选择当前空闲 GPU 和新的输出目录。
- 本地 `pytest -q dynamicbench/tests`：56 passed；Ruff 和 `git diff --check` 通过。
