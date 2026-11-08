# DynamicBench 当前版本与接口索引

更新日期：2026-11-08。本文覆盖当前传送带任务的代码、入口和数据接口；详细参数以配置文件及每次运行的 `metadata.json` 为准。后续修改直接更新本文，不在此累积历史版本。

## 1. 版本基线

| 项目 | 当前基线 |
| --- | --- |
| 仓库 / 分支 | [z-R-rr/dynamicbench](https://github.com/z-R-rr/dynamicbench) / `feat/conveyor-rtc-expert-mvp` |
| 功能代码提交 | `bdc88043f90b23d8a01533175f5c0b65be84cd33`；本文的文档提交可位于该提交之后 |
| 运行环境 | Isaac Sim `6.1.0.0`、Python `3.12.15`、Isaac Lab `v3.0.0-EA`（`ae37b028ea415c91ea2bc32609efcd759ed2b974`）；包版本 `isaaclab==17.0.2`、`isaaclab-physx==6.0.0` |
| 仿真边界 | 独立单环境传送带任务；CPU PhysX、RTX 相机；300 Hz 物理、30 Hz 动作控制 |
| 场景与构型 | 默认 RoboDojo 官方布局；X5、Piper、Franka 共用传送带前方安装位置来源；构型各自解析关节、EEF、TCP 与夹爪 |
| 物体 | 统一 `object` 接口：box / cylinder / sphere primitive，或经过单刚体检查的 USD |
| 调度参照 | LIBERO-Dynamic `b66cb6b6c3a4b52932cb96ca6ee66ab5fe5c51f4`；队列协议对齐，动作仍为本任务的绝对 EEF10 |
| 模型状态 | 已提供真实模型适配接口；已有 mock 调度验证，尚无匹配 checkpoint 的真实模型闭环评测结果 |

环境安装细节见 [6.1 迁移记录](isaacsim61_migration.md)。当前功能验证材料见 [前方布局验证](../results/front_layout_20261108.md)、[前方布局与物体集成验证](../results/front_object_integration_20261108.json)、[官方布局验证](production_layout_validation_20261108.md)、[物体配置验证](../results/object_config_validation_20261108.json)。各报告中的提交号和运行元数据定义其证据范围。

## 2. 入口与调用关系

```text
run_isaac61.sh <entry> [args]
  └─ run_isaac.sh → scripts/dynamicbench/<entry>.py
       ├─ cli.parser / launch → Kit 应用、参数、提交号
       ├─ cli.load_settings → 物体 / 官方布局 / 构型 / 渲染 / 场景配置
       └─ ConveyorCubeTask
            ├─ run_mvp / collect_expert → episode.run_episode
            │    ├─ ConveyorCubeExpert（专家采集）
            │    └─ RTCScheduler + MockChunkPolicy（旧版 schedule 诊断）
            └─ evaluate_policy → evaluation.run_evaluation
                 └─ LiberoScheduler + MockChunkPolicy 或 CheckedPolicy

动作执行：EEF10 → FixedRateController → solve_target / IK
         → 关节目标保持 → 原生物理步 → observe / outcome
数据输出：transition + trace → EpisodeRecorder → trajectory / metadata / manifest
```

`ConveyorCubeTask` 的物理推进由 `FixedRateController` 独占；它不是 Gym 的 `step(action)` 接口。`validate_mvp` 是旧版综合检查入口，新版 LIBERO 调度检查使用 `evaluate_policy --validate-scheduling`。

### 命令入口文件

以下路径相对仓库根目录；Shell 启动器接受 `run_mvp`、`collect_expert`、`validate_mvp`、`evaluate_policy` 四个 entry。

| 文件 | 功能与入口接口 |
| --- | --- |
| [run_isaac61.sh](../scripts/dynamicbench/run_isaac61.sh) | `bash … <entry> [args]`：选择隔离的 6.1 环境并校验版本，转交通用启动器 |
| [run_isaac.sh](../scripts/dynamicbench/run_isaac.sh) | `bash … <entry> [args]`：使用当前/指定 Python，设置依赖库路径后执行入口 |
| [run_mvp.py](../scripts/dynamicbench/run_mvp.py) | `main()`；`--test belt/expert/schedule`：单次传送、专家或旧版调度诊断 |
| [collect_expert.py](../scripts/dynamicbench/collect_expert.py) | `main()`；`--num-success`、`--max-attempts`、`--save-failures`：逐 seed 采集，带输出锁、成功目标和失败记录 |
| [evaluate_policy.py](../scripts/dynamicbench/evaluate_policy.py) | `main()`；`--execution-mode`、`--latency-source`、`--policy-factory`、`--policy-config`；`--validate-scheduling` 执行四种模式的 mock 验证 |
| [validate_mvp.py](../scripts/dynamicbench/validate_mvp.py) | `main()`；综合检查传送、静态/动态专家、旧版调度和数据读回 |
| [inspect_dataset.py](../scripts/dynamicbench/inspect_dataset.py) | `python … PATH [--video FILE --camera head/wrist]`：读取 episode 目录或包含 manifest 的数据目录，校验并可导出单 episode 视频 |
| [audit_reset_rgb.py](../scripts/dynamicbench/audit_reset_rgb.py) | `python … PATH [--output JSON]`：离线检查 reset 首帧与黄色方块投影；不是通用物体检测器 |
| [audit_visual_layout.py](../scripts/dynamicbench/audit_visual_layout.py) | 独立 `main()`，需 Isaac 环境；公共 CLI 加 `--visual-variant current/official_belt/official_belt_home`、`--reference-layout`，输出布局对比审计 |
| [check_libero_conformance.py](../scripts/dynamicbench/check_libero_conformance.py) | `python … REFERENCE --output JSON`：对固定提交的 LIBERO-Dynamic checkout 做 CPU 队列差分检查 |
| [prepare_isaac61.sh](../scripts/dynamicbench/prepare_isaac61.sh) | `bash …`：安装固定的 Sim 6.1 / Lab 3 环境，不是每次仿真的启动入口 |
| [prepare_server.sh](../scripts/dynamicbench/prepare_server.sh) | 旧版 5.1 服务器依赖准备；不用于更新 6.1 环境 |
| [requirements.txt](../scripts/dynamicbench/requirements.txt) | CPU 工具与测试依赖清单；完整 6.1 环境固定由安装脚本负责 |

公共任务参数由 `cli.parser()` 定义，主要包括 `--config`、`--embodiment x5/piper/franka`、`--piper-asset-root`、`--object-config`、`--layout-profile official/compact`、`--belt-source`、`--scene-source`、`--seed`、`--ticks`、`--output`、`--device`、`--render-gpu`。完整选项以该函数为准。

## 3. 文件职责与主要 Python 接口

表中的 `settings` 是解析后的配置字典，`root` 是仓库根路径；`…` 表示省略可选参数，完整签名见链接。以下列出模块间使用的接口，不列内部私有辅助函数。

### 配置、布局与场景

| 文件 | 功能 | 主要接口 / 输出 |
| --- | --- | --- |
| [cli.py](../dynamicbench/cli.py) | 公共参数、应用启动、配置解析与退出处理 | `parser(collect=False, compare_lift=False, evaluation=False)` → parser；`launch(p)` → `(args, launcher, commit)`；`load_settings(args)` → settings；`finish(args, status)` → 状态码并写 `exit_status.json`；`exception_details(exc)` → 诊断字典 |
| [layout.py](../dynamicbench/layout.py) | 解析官方机器人根位姿、头部相机和源文件信息 | `resolve_robodojo_layout(settings, root)` → settings 副本；`official_camera_quaternion(orientation)` → WXYZ |
| [robot_sources.py](../dynamicbench/robot_sources.py) | 读取 X5 官方资产配置：关节、TCP、夹爪、腕部相机 | `resolve_robot_asset_config(settings, root)` → settings 副本 |
| [embodiments.py](../dynamicbench/embodiments.py) | X5 / Piper / Franka 构型选择、初始 IK、资产转接 | `resolve_embodiment(settings, root, name="x5", piper_asset_root=None, *, prepare_initial_pose=True)`；`resolve_initial_pose(settings)` → settings；`articulation_config(rc, root)` → `ArticulationCfg`，并补充 `rc` 的 USD / body 路径 |
| [production_layout.py](../dynamicbench/production_layout.py) | official / compact 布局，官方传送带几何、X5 默认关节和被动臂 | `resolve_production_layout(settings, root)` → settings；`official_joint_defaults(path)` → 从源码 AST 读取的默认关节映射 |
| [objects.py](../dynamicbench/objects.py) | 统一物体配置、兼容旧 `cube` 字段、primitive 几何与抓取点 | `resolve_object_settings(settings, root, override=None)` → settings；`spawn_position(obj, belt_top)` → XYZ；`vertical_bounds(geometry, pose)` → `(bottom_z, top_z)`；`grasp_position(obj, pose)` → 世界 XYZ |
| [scene.py](../dynamicbench/scene.py) | 官方 Room / Background / Light / camera_stand / Ground 的配置及实例化 | `resolve_scene(settings, root)` → settings；`resolve_camera_stand(original, root)` → 支架配置；`spawn_scene(cfg, stage)` → `((room, background, lights, stand), audit)`；`spawn_official_ground(cfg, stage)` → audit；`room_physics_snapshot(stage, root)` → 碰撞/刚体记录 |
| [rendering.py](../dynamicbench/rendering.py) | 读取官方渲染默认值、映射 6.1 渲染设置并检查生效值 | `resolve_render_settings(root)` → 渲染配置与来源；`isaac61_global_settings(render)` → RTX 配置；`audit_render_settings(render)` → 检查报告 |

`load_settings()` 的解析顺序是：任务 YAML → 物体覆盖 → 官方布局 → 构型（暂不解初始 IK）→ 渲染 → CLI 覆盖 → production layout → 初始 IK → scene。新增构型的初始 IK 使用 production layout 解析后的几何及 metadata 带面高度；生成 USD 后的实测带面高度由场景构建阶段读取，供物体初始化和专家使用。

### 环境、物理与控制

| 文件 | 功能 | 主要接口 / 输出 |
| --- | --- | --- |
| [envs/base_env61.py](../dynamicbench/envs/base_env61.py) | 6.1 的 SimulationContext / InteractiveScene 生命周期与物理计数 | `BaseEnv61(config, app, *, interactiveSceneCfg)`；`sim_step(render=False)`、`render()`、`close()`；子类场景/物理钩子 |
| [envs/conveyor_cube_task.py](../dynamicbench/envs/conveyor_cube_task.py) | 组装机器人、物体、传送带、相机和接触传感器，给出观测与成败 | `ConveyorCubeTask(settings, app, seed=42, belt_speed=None)`；`reset(seed=None, options=None)`；`observe()`；`solve_target(action)`；`set_joint_target(q)`；`before_physics_step()`；`sim_step(render=False)`；`outcome(observation, tick)`；`audit_assets(asset_root, robot=None)` |
| [envs/fixed_rate_controller.py](../dynamicbench/envs/fixed_rate_controller.py) | 唯一控制时钟所有者、动作校验、IK 失败保持、关节目标零阶保持 | `FixedRateController(backend, max_ik_failures=5, *, evaluation_timing=False)`；`observe()` → 带时间戳观测；`step(action, *, hold=False, physics_steps=10)` → transition；`ControlFailure.transition` 保留失败转移 |
| [envs/physics_clock.py](../dynamicbench/envs/physics_clock.py) | 整数物理计步与采集/评测两种时间索引 | `integral_ratio(dt, hz, expected=10)`；`PhysicsClock`、`EvaluationClock` 的 `advance()`、`stamp()`、`sim_time`、`control_tick` |
| [envs/kinematics.py](../dynamicbench/envs/kinematics.py) | URDF 正解、含关节限制的逆解 | `URDFArmIK(path, base_position, base_rotation, joint_names, end_link=None)`；`fk(q)` → 4×4 变换；`solve(position, rotation, current_q, limits=None)` → q 或 `None` |
| [envs/poses.py](../dynamicbench/envs/poses.py) | EEF10 编解码、旋转与范围检查 | `encode_eef10(position, rotation_matrix, gripper)` → 10 维动作；`decode_eef10(value)` → `(xyz, R, grip)`；`validate_eef10(value)` → 经校验副本；`quat_wxyz_matrix(q)` → R |
| [envs/object_asset.py](../dynamicbench/envs/object_asset.py) | 构建 primitive / USD spawn 配置，检查 USD 单刚体与碰撞结构 | `object_spawn_config(obj)` → spawn 配置；USD 分支会补充 `obj` 的解析几何和资产依赖 hash |
| [envs/runtime.py](../dynamicbench/envs/runtime.py) | Sim 5.1 / 6.1 张量、四元数与状态写入 API 边界 | `sim_version()`、`runtime_metadata()`；`tensor(value)`、`lab_quaternion(wxyz)`、`public_pose(value)`；`joint_state(robot, position, velocity)`、`root_state(obj, state)`、`position_target(robot, target)`、`velocity_target(robot, target)` |
| [envs/fixtures61.py](../dynamicbench/envs/fixtures61.py) | 用 USD API 实现 6.1 固定场景组件 | `Room(prim_path, usd_path, instance_config)`、`Background(prim_path, inst_config)`、`Light(prim_path, light_type, config)`；`transform(prim, position, wxyz, scale=…)`；`fixed_pair(values)` 校验固定范围 |
| [envs/scene_paths.py](../dynamicbench/envs/scene_paths.py) | 固定单环境 prim 路径，避免外部资产内嵌 Cube 触发错误回调 | `ENV_PRIM_PATH`、`CUBE_PRIM_PATH`；后者也用于非方块物体，属于兼容路径名 |

### 专家、评测、调度与数据

| 文件 | 功能 | 主要接口 / 输出 |
| --- | --- | --- |
| [experts/conveyor_cube_expert.py](../dynamicbench/experts/conveyor_cube_expert.py) | 使用 oracle 的抓取状态机与接触锚定抬升 | `ConveyorCubeExpert(settings, belt_top)`；`action(obs)` → EEF10；`change(phase)`；暴露当前阶段和失败原因 |
| [episode.py](../dynamicbench/episode.py) | 统一专家/传送诊断/旧版 schedule 的 episode 循环和元数据 | `run_episode(env, *, test="expert", ticks=600, mode="no_latency", latency=0.25)` → `(recorder, metrics)`；`metadata(env, metrics, commit)` → 运行元数据 |
| [runtime/evaluation.py](../dynamicbench/runtime/evaluation.py) | 新版评测循环，显式记录同步等待转移 | `run_evaluation(env, policy, *, ticks=300, mode="async_queue", horizon=32, threshold=16, latency_source="fixed", latency=0.25, samples=None, seed=42)` → `(recorder, metrics)` |
| [runtime/libero_scheduler.py](../dynamicbench/runtime/libero_scheduler.py) | LIBERO-Dynamic 队列协议、单推理 worker、延迟与请求追踪 | `LiberoScheduler(policy, *, mode="async_queue", horizon=32, threshold=16, latency_source="fixed", latency=0.25, samples=None, seed=42, control_hz=30, worker_timeout=120)`；`tick(k, observation)` → `(action_or_None, info)`；`close()`；`blend_eef10(old, new)` → 旋转重新正交化后的融合动作 |
| [runtime/policy_contract.py](../dynamicbench/runtime/policy_contract.py) | 真实模型构型/动作协议及来源声明检查 | `CheckedPolicy(policy, config, *, embodiment="x5")`；`validate_config(config, *, embodiment="x5")`；`predict(observation, request_tick, horizon)`；`predict_rtc(observation, request_tick, horizon, **kwargs)` |
| [runtime/mock_chunk_policy.py](../dynamicbench/runtime/mock_chunk_policy.py) | 确定性动作块，用于延迟/队列验证 | `MockChunkPolicy.predict(observation, request_tick, horizon)` → `(H, 10)`；不是学习模型或抓取专家 |
| [runtime/rtc_scheduler.py](../dynamicbench/runtime/rtc_scheduler.py) | 保留旧版 MVP 调度语义，供 `run_episode(test="schedule")` 使用 | `QueuedAction`；`RTCScheduler(mode="rtc_async", latency=.25, horizon=32, replan_stride=16, commit_ticks=None)`；`tick(k, observation, policy)` → `(action, info)` |
| [data/recorder.py](../dynamicbench/data/recorder.py) | 收集连续 transition，原子保存 episode 后更新 manifest | `EpisodeRecorder.append(result, trace)`；`arrays()` → NPZ 字段字典；`save(root, name, metadata, video=True)` → manifest entry；`jsonable(v)` → JSON 可序列化值 |
| [data/dataset_reader.py](../dynamicbench/data/dataset_reader.py) | 严格检查时序、动作、物体几何及持续成功条件 | `validate(arrays, metadata)` → summary；`read_episode(path)` → `(arrays, metadata, summary)`；`export_video(arrays, path, camera="head")` |
| [data/motion_audit.py](../dynamicbench/data/motion_audit.py) | 从轨迹和 trace 量化接触后位移与 lift 相对滑动 | `contact_motion(arrays, trace, *, force_threshold=1.0)`、`lift_motion(arrays, trace)` → 指标字典或 `None` |
| [data/reset_image_audit.py](../dynamicbench/data/reset_image_audit.py) | 首帧黄色方块位置与相机投影一致性检查 | `audit_reset_image(image, cube_position, camera, tolerance_px=10.0)` → audit |

包目录中的 `__init__.py`（[根包](../dynamicbench/__init__.py)、[envs](../dynamicbench/envs/__init__.py)、[experts](../dynamicbench/experts/__init__.py)、[runtime](../dynamicbench/runtime/__init__.py)、[data](../dynamicbench/data/__init__.py)）是包标记，不提供额外执行入口。

## 4. 跨模块数据接口

### 动作、观测与 transition

- **EEF10**：`[x, y, z, R[:,0] 的三项, R[:,1] 的三项, gripper]`；世界系绝对 EEF 位姿，夹爪 `-1` 关闭、`+1` 打开。X5 / Piper 的 EEF 为 `link6`，Franka 为 `panda_hand`。它不等同于抓取 TCP 位置；专家按构型 TCP 偏置换算。公共 pose7 使用 `[xyz, qw, qx, qy, qz]`。
- **`env.observe()`**：`eef10`、`qpos`、`qvel`、`head`、`wrist`、`oracle_debug`。图像为 `uint8[H,W,3]`，关节向量长度由构型决定。`oracle_debug` 含物体位姿/速度、上下界/抓取点、双指接触力及相对位置；旧 `cube_*` 名称保留兼容。
- **`controller.observe()`**：在上述观测上增加 `sim_time`、`physics_step`、`control_tick`。`env.observe()` 本身不添加这些时间字段。
- **`controller.step()` 返回值**：`{observation, action, joint_target, next_observation, control_error}`。记录的是实际提交执行的动作；IK 失败时保持上次目标，连续失败抛出带 transition 的 `ControlFailure`。
- **环境返回约定**：`reset()` 返回 `None`；`solve_target(action)` 返回 `(q, None)` 或 `(None, error)`；`outcome(obs, tick)` 返回 `(success, failure_reason_or_None)`。reset 后须重建 controller；绕开 controller 推进物理会触发所有权检查。

### 真实模型接口

CLI 使用 `--policy-factory package.module:function --policy-config CONFIG.json`。加载器调用 `factory(config)` 得到 policy，再由 `CheckedPolicy` 包装。必需方法：

```python
policy.predict(observation, request_tick, horizon)  # 返回 shape (horizon, 10)
```

传给模型的 observation 是独立副本，只允许 `eef10/qpos/qvel/head/wrist`；不包含 oracle、场景对象或可变仿真张量。模型适配器负责图像/状态预处理、归一化及输出到绝对 EEF10 的转换。

配置必须声明：`robot`、`action_frame`、`action_representation="absolute_eef10"`、`rotation_6d="first_two_columns_concatenated"`、`gripper="minus1_closed_plus1_open"`、`control_hz=30`，以及非空 `checkpoint`、`checkpoint_sha256`（64 位小写十六进制）、`normalization`、`training_data`、`model_class`。

| `--embodiment` | `robot` | `action_frame` |
| --- | --- | --- |
| `x5` | `arx_x5` | `world_link6` |
| `piper` | `piper` | `world_link6` |
| `franka` | `franka_panda` | `world_panda_hand` |

`CheckedPolicy` 检查声明及接口，不自行读取 checkpoint 文件重算 hash；这些来源信息需由调用方真实提供。`is_real_model=True` 表示进入了模型适配入口，不代表该模型已通过任务验证。

只要模型声明 `supports_rtc_conditioning=True`，就必须实现以下方法；仅在 `rtc` 模式下调用：

```python
policy.predict_rtc(
    observation, request_tick, horizon,
    prev_chunk_left_over=leftover,       # (N, 10)，未执行的原始、未融合动作
    inference_delay_steps=delay_steps,  # ceil(latency * control_hz)，实测延迟时为 None
)  # 返回 shape (horizon, 10)
```

没有该声明时，`rtc` 调用普通 `predict`，仅执行 RTC 队列规则。调度器验证输出形状及每行 EEF10。更多说明见 [LIBERO 调度说明](libero_scheduling.md)。

### 调度与时间接口

`LiberoScheduler.tick(k, observation)` 的 `k` 从 0 连续递增，返回动作或 `None`（关节保持）以及 `info`。`info` 字段为 `control_step`、`queue_before`、`queue_after`、`underflow`、`sync_latency_seconds`、`action_source`（`queue/finite_joint_hold`）、`chunk_id`、`action_index`；`scheduler.requests` 保留请求与响应处理记录。

| 模式 | 队列与等待语义 |
| --- | --- |
| `no_latency` | 队列耗尽时请求新 chunk，模拟延迟为 0 |
| `sync_hold` | 队列耗尽时请求；返回等待时长，由执行层保持关节目标并推进物理，不重复计入队列延迟 |
| `async_queue` | 按阈值预取，丢弃请求后实际执行动作数对应的前缀（欠载不计）；重叠段按旧/新 0.3/0.7 融合，替换队列 |
| `rtc` | 按阈值预取，按响应到达延迟的控制步数丢弃前缀（包含欠载），直接替换队列、不融合 |

`fixed/empirical` 使用配置或采样延迟。`measured_wallclock` 测量 worker 从调用策略到 CPU 动作转换、校验完成的实际耗时；单独保留 `policy_call_wall_seconds`，不包含观测拷贝、worker 启动和外部排队时间。异步推理未结束时，控制边界受该请求的墙钟进度约束，防止仿真跑过结果可用时刻；仿真较慢时按测得耗时推进到相应控制边界，不再额外等待墙钟。RTC 使用实际 `release_tick - request_tick` 裁剪前缀，并同时记录理论可用时刻、实际生效时刻和晚到步数；普通异步仍按已执行队列动作数裁剪。

普通动作包含 10 个物理步；同步等待按 300 Hz 向上量化，尾段可有 1–9 个物理步。只有 `evaluation_timing=True` 且 `hold=True` 允许这种短转移。新版数据的 `control_tick` 是记录转移序号，调度动作序号在 trace 的 `control_step`；不得用两者互相替代。

`validate_measured_clock` 是独立诊断入口：用真实墙钟睡眠的 mock worker 验证四种调度模式，并在额外减慢物理执行时复测异步/RTC；保存逐请求时间、物理并发步数、队列裁剪、严格数据校验与专家回归证据，不标记为真实模型测试。

`run_evaluation()` 返回 recorder 和指标，不自行保存文件；入口脚本负责保存。指标包括任务结果、`control_actions`、`recorded_transitions`、`physics_steps`、`sim_time`、同步等待步数、欠载、请求记录和策略来源。

### 存储接口

每个 episode 含 `trajectory.npz`、`metadata.json`、`trace.jsonl`、可选 `head.mp4`；父目录的 `manifest.jsonl` 在 episode 完整保存后更新。已有同名 episode 会报错，不覆盖。

| 数据 | 约定 |
| --- | --- |
| 动作 / 观测 | N 条动作、N+1 条观测；NPZ 核心列为 `action.eef10`、`action.joint_target`、`observation.state.*`、`observation.images.*`、`timestamp.sim`、`physics_step`、`control_tick`、`done`、`oracle_debug.*` |
| 专家 / 旧版采集 schema | `dynamicbench.eef10.columns.v1`，固定间隔 |
| 新版评测 schema | `dynamicbench.eef10.eval.v2`，允许显式同步等待转移，不能假定所有行间隔相同 |
| 物体 schema | `dynamicbench.object.v1`；元数据保留物体形状/几何、配置、来源与 hash，旧 `cube_*` 轨迹别名兼容 |
| 运行来源 | `metadata(env, metrics, commit)` 保存 source commit、解析后的 settings、实际运行版本、seed、构型、物体及结果 |

详细字段见 [数据格式](data_schema.md) 和 [物体配置接口](object_configuration.md)。

## 5. 配置文件与上游依赖

| 文件 | 职责 / 消费方 |
| --- | --- |
| [configs/pick_cube_from_conveyor.yaml](../dynamicbench/configs/pick_cube_from_conveyor.yaml) | 主任务配置：`scene/layout/robot/belt/object/expert` 等，由 `cli.load_settings()` 解析 |
| [configs/sim_300hz_control_30hz.yaml](../dynamicbench/configs/sim_300hz_control_30hz.yaml) | 时钟约定说明文件；当前控制实现仍由 `PhysicsClock` / controller 固定并检查，不作为公共 CLI 的独立仿真配置加载 |
| [configs/objects/cylinder.yaml](../dynamicbench/configs/objects/cylinder.yaml) | `--object-config` 圆柱示例，仅顶层 `object` |
| [configs/objects/sphere.yaml](../dynamicbench/configs/objects/sphere.yaml) | `--object-config` 球体示例，仅顶层 `object` |
| [configs/objects/robodojo_block.yaml](../dynamicbench/configs/objects/robodojo_block.yaml) | `--object-config` 官方积木 USD 示例，仅顶层 `object` |
| [configs/robodojo_quality.kit](../dynamicbench/configs/robodojo_quality.kit) | 固定的质量设置来源，由 rendering 解析、审计 |
| [configs/LICENSE.isaaclab-quality.txt](../dynamicbench/configs/LICENSE.isaaclab-quality.txt) | quality 配置对应许可证，无执行接口 |
| [env_cfg/robot/dual_x5.yml](../env_cfg/robot/dual_x5.yml) | 官方机器人根位姿来源；三种构型继承前方安装位来源 |
| [env_cfg/camera/camera_config.yml](../env_cfg/camera/camera_config.yml)、[template.py](../env_cfg/camera/template.py) | 头部相机配置与 D435 模板；X5 腕部挂载还读取资产配置 |
| [env_cfg/scene/conveyor.yml](../env_cfg/scene/conveyor.yml) | 官方房间、背景、相机支架配置来源 |
| [env/robot_manager/robot_config/x5.py](../env/robot_manager/robot_config/x5.py)、[franka.py](../env/robot_manager/robot_config/franka.py) | 官方 articulation 与关节配置来源 |
| [env/environment/base_env.py](../env/environment/base_env.py) | 上游渲染默认值来源及旧版环境基类；6.1 使用独立 `BaseEnv61` |
| [env/camera_manager/capture/render_sync.py](../env/camera_manager/capture/render_sync.py) | 相机捕获时的渲染同步辅助 |

`Assets/` 中的机器人配置、URDF / USD、Room / Background 和官方布局 JSON 是运行时资产依赖；Piper 还使用指定或相邻 RoboTwin 的资产目录。解析器将使用到的源文件与 hash 写入运行配置。大型资产、生成的 Piper USD 和隔离运行环境不属于本索引中的源代码文件。

## 6. 测试文件索引

统一接口：仓库根目录执行 `python -m pytest dynamicbench/tests -q`。这些是 CPU 契约测试；仿真运行证据另见第 1 节链接。

| 文件 | 检查范围 |
| --- | --- |
| [test_clock.py](../dynamicbench/tests/test_clock.py)、[test_zoh.py](../dynamicbench/tests/test_zoh.py) | 物理/控制时钟、零阶保持、物理所有权与失败处理 |
| [test_poses.py](../dynamicbench/tests/test_poses.py) | EEF10、旋转和数值合法性 |
| [test_runtime_boundary.py](../dynamicbench/tests/test_runtime_boundary.py) | Sim 5.1 / 6.1 四元数和张量接口边界 |
| [test_scene_paths.py](../dynamicbench/tests/test_scene_paths.py) | 场景 prim 路径隔离 |
| [test_layout.py](../dynamicbench/tests/test_layout.py)、[test_production_layout.py](../dynamicbench/tests/test_production_layout.py) | 官方相机/安装布局、production profile、传送带来源 |
| [test_official_sources.py](../dynamicbench/tests/test_official_sources.py)、[test_upstream_defaults.py](../dynamicbench/tests/test_upstream_defaults.py) | 官方 TCP、相机、场景与渲染默认值读取 |
| [test_embodiments.py](../dynamicbench/tests/test_embodiments.py) | 构型、前方安装继承、初始位姿与 FK/IK |
| [test_objects.py](../dynamicbench/tests/test_objects.py) | 物体配置校验、几何和旧配置兼容 |
| [test_object_contract.py](../dynamicbench/tests/test_object_contract.py) | 通用物体的接触锚定与倾斜几何成功判据 |
| [test_expert.py](../dynamicbench/tests/test_expert.py)、[test_motion_audit.py](../dynamicbench/tests/test_motion_audit.py) | 专家状态机、接触/抬升运动审计 |
| [test_libero_scheduler.py](../dynamicbench/tests/test_libero_scheduler.py) | 新版调度、延迟、RTC、等待转移与队列协议 |
| [test_policy_contract.py](../dynamicbench/tests/test_policy_contract.py) | 模型来源声明、构型和动作接口 |
| [test_rtc.py](../dynamicbench/tests/test_rtc.py) | 旧版 MVP RTC 队列 |
| [test_dataset.py](../dynamicbench/tests/test_dataset.py)、[test_reset_image_audit.py](../dynamicbench/tests/test_reset_image_audit.py) | 数据保存/严格读回、schema、首帧投影一致性 |
| [dynamicbench-tests.yml](../.github/workflows/dynamicbench-tests.yml) | GitHub Actions：Python 3.11 / 3.12 CPU 测试矩阵 |

维护约定：新增/删除模块或修改公开签名、动作/观测/schema 时，同步修改对应行和第 4 节；功能代码更新后刷新基线提交。运行验证保留在独立报告中，本文只链接，不把历史通过结果改写为新版本已通过。
