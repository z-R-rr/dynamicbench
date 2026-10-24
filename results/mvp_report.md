> 2026-10-25 接触控制修正：前一轮漏检了 close_gripper 接触后的速度反馈自我追逐，旧版在此阶段移动约 12 cm。当前默认 contact_anchor 已将接触后 EEF 峰值水平偏移降至约 0.35 mm，三个动态 seed 通过；原装 USD 10 项回归和 36 项 CPU 测试通过。详见 docs/contact_anchor.md；6.1 仅评估、未升级。以下历史结果保留原始判据与 SHA。

> 2026-10-25 最新补充：已加入实际夹持中心换算的当前方块 XY 跟踪，并复用原生 Room/Background/Light 渲染。13 项验收、三个动态 seed 和 33 项 CPU 测试通过；原始方案 A 的失败及位移单位复核见 docs/lift_tracking_scene.md。以下早期统计按各自 source_commit 保留。

# DynamicBench MVP 实测报告（2026-10-24，2026-10-25 补充修复）

已完成原生 PhysX 传送、静态抓取、0.03 m/s 动态抓取、三种模拟延迟模式及专家数据采集。
这些结果来自实际 Isaac Sim 执行和保存的原始状态/RGB，不是单元测试中的假后端。
默认仍使用简化接触皮带。2026-10-25 已修复原装 USD 初始化，原装分支也通过 10/10 项物理验收和三个动态抓取 seed。

## 仓库与代码边界

- 实际 Fork：https://github.com/z-R-rr/dynamicbench ，父仓库为 RoboDojo-Benchmark/RoboDojo。
- 分支：`feat/conveyor-rtc-expert-mvp`；Draft PR：https://github.com/z-R-rr/dynamicbench/pull/1 ，目标为本人 Fork 的 main。
- 上游起点：`f57e1633ffed59761bc8e568cfe7530c4b33381c`。
- 初次完整物理验收：`4f60d8c670d569dc3a964d8e43a280ef1cea7f38`。
- 独立 collect_expert 实采：`82734179907a686c31f603f87223e103b5e118b2`。
- 原版布局完整复验：`c1e4faf2afc400ca91b03af3c1abe220fc0fbb7e`，10/10 项通过；见 physical_acceptance.json。
- 本报告是随后文档提交；交付 HEAD 请执行 `git rev-parse HEAD`，各次数据保留其真实运行 SHA。
- 本地 `/Users/riri/code/dynamicbench`；服务器 `/home/ubuntu/Workspace/dynamicbench`。

新增 dynamicbench/（控制、IK、专家、调度、数据、测试）、scripts/dynamicbench/、独立任务入口、docs/、results/ 和 CPU CI。
原有文件仅修改 .gitignore；没有删除文件。原 env、env_cfg、src/eval_client、42 个任务实现/配置、注册器、README、LICENSE 和子模块指针均保持原样。
默认 .004 s / 25 Hz 与原 80/20 插值路径有回归检查；未运行全部 42 个旧任务的 GPU 回归，不能把源码保留等同于运行验证。
未修改现有 OpenWAM / RoboDojo 项目、下载资产源文件或其他用户任务；无强推。

## 物理执行与时间语义

每 tick 先取 observation[k]，下发绝对 EEF10 目标，用当前关节状态作为 URDF IK seed，更新有限速度的原生关节驱动，
保持同一目标并调用原 BaseEnv.sim_step 恰好十次。300 Hz 物理、30 Hz ZOH；没有到位等待、TOPP、80/20 插值、夹持焊接或运行期方块 pose/velocity 写入。
父类会强制 GPU dynamics，因此本任务子类明确恢复并检查 CPU PhysX；渲染仍用 GPU。
方块禁用休眠，让静止运动学皮带上的接触速度变化持续作用；位置和关节状态写入仅用于 reset。

成功要求同一动态方块的**底面**高于带面 5 cm，并连续 15 个控制 tick（0.5 s）同时存在左右夹指对该方块的接触。
严格 reader 再次从保存的数据检查高度、双侧接触、10:1 步数、时间戳和 N 动作/N+1 观测。
渲染在冻结物理状态下等待两次相机交付，并断言物理计数未改变；渲染/编码耗时不计入仿真时间。

| 初次完整验收 | 实测结果 |
| --- | --- |
| 速度 0，100 tick | x 漂移 0.00002235 m |
| 速度 .03，100 tick | x 位移 0.09999026 m，期望 .10000000 m |
| 速度 .06，100 tick | x 位移 0.19997287 m，期望 .20000000 m |
| 静态专家 seed42 | 成功，90 tick / 3.0 s |
| 动态专家 .03，seed42/43/44 | 3 次尝试、3 次成功，96/95/95 tick |
| 独立 collect_expert | 3 次尝试、3 次成功；共 286 tick，成功平均时长 3.17778 s |
| no_latency/sync_hold/rtc_async | 各 100 tick，均通过 |

3/3 是这三个固定 seed 的 MVP 结果，不能解释为广泛分布上的可靠成功率。
同一批 seed 的重复复验单独记录，不把重复 seed 合并为更多独立样本。
0.06 m/s 仅验证输送，未验证该速度抓取；负向速度不支持。

250 ms 模拟延迟按 ceil(.25×30)=8 tick 量化为 266.667 ms（+16.667 ms）。
两个延迟模式的初始等待期间，方块仍前进 0.00798871 m。
RTC 在 tick16..23 继续执行旧 chunk，真实末端在该段移动 0.00233205 m；与无延迟轨迹的最大位置差为 0.00681290 m，sync_hold 为 0.01313957 m。
调度对照中机械臂悬在皮带上方，不与方块交互，因此三种模式方块轨迹相同；差异体现在真实机器人轨迹。
固定延迟 mock 在提交时立即算好 chunk、按仿真 tick 延后释放；**未实现/实测真正后台 GPU 模型推理或 OpenWAM**。
sync_hold 使用有限驱动保持，并在等待后消费完整旧观测 chunk；与 LIBERO-Dynamic 的连续子 tick 等待并非完全相同。

## 相机、布局与训练数据

按用户要求直接读取 RoboDojo：

- `env_cfg/robot/dual_x5.yml` 的机器人 0：基座 [-.3,-.45,.765]，四元数来自原配置并归一化；保留单臂。
- `env_cfg/camera/camera_config.yml` 的 cam_head：位置 [0,-.41,1.308]，Euler [30,0,0] 度。
- `env_cfg/camera/template.py` 的 GEMINI_345LG：640×480，焦距 10，水平/垂直 aperture 22.212/14.266。

这些位置和镜头参数没有在新 YAML 中重复硬编码；数据 metadata 保存解析结果及原配置 SHA-256。
初次验收的旧头部相机为斜侧视角；旧腕部视角被遮挡。移动镜头到 link6 的 [.10,0,.08] 后，
独立采集的真实腕部 RGB 已能看到夹指和方块。旧数据保留作历史证据，不混入最终视角的数据集。

每个 episode 包含 trajectory.npz（原始头部/腕部 RGB、实际状态、命令和 privileged debug）、metadata.json、trace.jsonl、head.mp4。
N 个动作对应 N+1 个观测，包含最终尾帧。GT 仅在 oracle_debug，默认策略 snapshot 不包含方块真值。
EEF10 是 world link6 xyz + 旋转矩阵前两列拼接 + [-1 闭合,+1 张开] 夹爪值，schema 明确版本化；没有假定其与 Piper/OpenWAM 旋转编码天然一致。
数据/资产/视频没有提交 Git；Git 只保存小型实测 JSON、manifest 和文档。

服务器证据：

```text
/home/ubuntu/Workspace/dynamicbench/outputs/acceptance_awake/
/home/ubuntu/Workspace/dynamicbench/datasets/conveyor_cube_v0_mask12/
/home/ubuntu/Workspace/dynamicbench/outputs/acceptance_robodojo_view/
/home/ubuntu/Workspace/dynamicbench/datasets/conveyor_cube_v0_robodojo_view/
```

本地交付目录中的 acceptance_robodojo_view/ 保存最终原版视角数据及头部/腕部视频；manifest 列出实际 episode。
最终物理复验与回读详情分别见 physical_acceptance.json、readback_validation.json。
只含三个成功动态 episode 的训练导出位于本地 dataset_conveyor_cube_v0/，并提供同名 ZIP；
这些是从最终验收原样导出的真实数据，source_commit 和成功标签均未改写。

## 环境、资产与尚未覆盖项

Ubuntu 22.04.5，驱动 580.95.05；Python 3.11.16，Isaac Sim 5.1.0.0，Isaac Lab 包 0.54.3 / experience 2.3.2 / checkout afca7b09，
Torch 2.7.0+cu128，NumPy 1.26.0。环境为 /home/ubuntu/miniconda3/envs/RoboDojo。
资产软链接至现有 39 GB 目录 /home/ubuntu/Workspace/RoboDojo/Assets；未改源资产。
资产仓库 SHA cfb06d1dadcdf03ccc923102b0e3f3b0a68dfc43。

| 资产 | SHA-256 |
| --- | --- |
| ARX.usd | 5723a835bf5627a935a8dabb3a44447725bd1465243ce791e221b3e990121d63 |
| X5A.urdf | 7aaa308a1a5b6a213d914db86ba09ddd6252f25cb498ac9a148bfa6c7166a423 |
| conveyor/00001/object.usdz | 0aae06620ed290d1204ce4fc32cbdd2ec42027fe94d99d545b5d5a9c03ede612 |

原传送带 USD 存在原生 PhysxSurfaceVelocityAPI 与运动图，不能声称资产没有运动能力。
此前原装分支报 RigidObject 缺少 _data 的根因现已定位并修复：禁用原装演示方块
/World/envs/env_0/OriginalConveyor/Cube 时，Isaac Lab 的删除回调用 env_.*/Cube 误匹配本任务方块，
取消了它的 PLAY 初始化订阅。本任务固定单环境，因此改用精确路径 /World/envs/env_0/Cube。
物理初始化继续由原生 RigidObject 执行；源 USD、Isaac Lab 和旧任务均未修改。
正式代码 SHA 为 1a0163ae0f60848849fa2eb153d0c83d6abb9951，GPU3 渲染 / CPU PhysX。
原装 USD：.03 输送 100 tick 的位移 0.0999726504 m；静态抓取成功；.03 动态抓取 3/3；
三种 RTC 模式通过。全部 10 个 episode 已严格回读、解码核对头部视频帧数。
详细调用链与复现命令见 docs/original_usd_fix.md，实测 JSON 见 physical_acceptance_original_usd.json。
原装结果单独保存在 /home/ubuntu/Workspace/dynamicbench/outputs/acceptance_original_usd_20261025，
既有 primitive 数据不重新标记为原装 USD 数据。

繁忙 GPU 0 的仅约 0.3 GB 空余会使 SDK 的跨卡自检、默认 CUDA/PhysX/Warp 上下文失败，即使 renderer 指定 GPU 2。
单独跳过启动自检不足以解决全部默认上下文分配。独立采集通过 CUDA_VISIBLE_DEVICES=1,2 与 GPU1 渲染完成；
最终复验将物理渲染 GPU 与 CUDA 重编号显式分开，在最空闲的物理 GPU2 上运行。
这些是本进程设置，没有更改系统 IOMMU、驱动或其他作业。失败进程仅按核对过的自有 PID 终止。
libGLU 依赖通过 apt-get download / dpkg-deb 解压到新仓库 outputs/deps，无 sudo。

本地、服务器和 GitHub CPU CI 已通过 30 个测试；源码 diff、Ruff、编译和 shell 语法检查见交付记录。
完整 42 旧任务运行、真实 OpenWAM/异步 GPU 推理及 .06 抓取未验证。
完整安装、资产准备、运行、采集、读数和测试命令见 [quickstart_dynamicbench.md](../docs/quickstart_dynamicbench.md)。
