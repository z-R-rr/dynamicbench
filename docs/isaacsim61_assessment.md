# Isaac Sim 6.1 升级评估（2026-10-25）

结论：可以作为独立环境迁移目标，但当前 RoboDojo 不能只替换 isaacsim 包后直接运行。
按用户本轮选择，仅完成评估；没有安装 6.1、替换现有环境或升级驱动。
接触后自我追逐来自专家状态机，修复和验证在现有 5.1 环境执行。

## 已核验的服务器与版本

| 项目 | 实际服务器 | 6.1 迁移判断 |
| --- | --- | --- |
| 系统 | Ubuntu 22.04.5 x86_64 | 满足官方列出的操作系统范围 |
| GPU | 四张 RTX 4090，每卡 24 GB | 满足官方 RTX/显存基础要求；复杂场景仍需实测 |
| 驱动 | 580.95.05 | 达到 Lab v3.0.0-EA 安装页所建议的 Linux 起点；Sim 6.1 要求页另列 595.58.03 为其测试版本，并非据此判定当前驱动一定不可用 |
| 磁盘 | 根分区剩余约 5.7 TB | 可容纳独立安装 |
| 当前 Python | 3.11.16 | 6.1/Lab 3.0 需新 Python 3.12 环境 |
| 当前 Sim | 5.1.0.0 | 迁移目标 6.1.0.0 |
| 当前 Lab | 包 0.54.3，experience 2.3.2，checkout afca7b09 | 对应 6.1 的明确标签为 v3.0.0-EA（ae37b02）；不要盲目跟随已移向新 Sim 版本的 main/release 分支 |

版本组合来自 [Isaac Lab 3.0 EA 发布说明](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-EA)
与 [固定标签安装文档](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html)。
GPU/系统和测试驱动见 [Isaac Sim 6.1 要求](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)。
主机基础条件符合不等于已通过 6.1 启动或任务回归。

## 本仓库需要迁移的接口

| 当前调用位置 | 必须处理的内容 |
| --- | --- |
| dynamicbench/envs/conveyor_cube_task.py、poses.py、layout.py，以及 X5 初始化 | Lab 3.0 四元数接口由 WXYZ 改为 XYZW；在边界显式转换。不要全局交换 USD Gf.Quat 或历史数据的分量。EEF10 的旋转 6D 本身不需要改语义 |
| conveyor_cube_task.py 中 robot/cube/camera/contact 的 data 读取 | Lab 3.0 数据属性使用 ProxyArray，检查 `.torch`/`.warp` 与 CPU/Numpy 转换，避免旧 Tensor 假设 |
| reset 中 write_joint_state_to_sim / write_root_state_to_sim | 迁移到区分索引与掩码的写入 API；写状态仍仅限 reset |
| env/environment/base_env.py、isaac/direct_rl_env.py 的封装 | 对照新的后端/配置接口核查自定义 sim_step、reset、scene.update，以及 300/30 Hz 单一推进所有者 |
| dynamicbench/cli.py 的 AppLauncher 和 _sim_app_config | 检查 6.1 GPU 选择和启动接口，重新验证 CUDA/Vulkan 映射及相机帧同步 |
| 原生 X5 actuator、CameraCfg、ContactSensorCfg 和场景组件 | 核查工厂/后端配置、关节和刚体索引、接触过滤器、图像交付及矩阵维度 |
| SurfaceVelocityAPI 传送带 | 首轮仍选择 Isaac Sim PhysX，以速度 0/.03/.06 的实际位移重新验收；切换 Newton 是独立实验，不能假设该 PhysX API 自动获得同样语义 |

官方 [2.x → 3.0 迁移指南](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/migration/migrating_to_isaaclab_3-0.html)
说明四元数、ProxyArray、写入方法及后端变化。上表右列中涉及本地封装的工作项属于本仓库源码审计结论，尚未运行迁移代码。

## 建议迁移顺序与验收

在独立 checkout/环境固定 Sim 6.1.0.0 + Lab v3.0.0-EA + Python 3.12；复用只读资产。
先运行官方空场景/关节与相机 smoke，再迁移本任务适配边界。官方固定标签的 uv 安装流程优先，
不要对正在使用的 RoboDojo Conda 环境执行 pip 升级。官方 Python 安装包命令参见
[Sim 6.1 Python 安装](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/install_python.html)。

先检查初始 FK 与 USD 一致、机器人/相机朝向正确，再跑传送带、静态抓取、三个动态 seed、
接触后横移指标、三种模拟延迟、完整数据回读。新旧版本按各自环境与代码 SHA 保存数据，
不能以成功启动代替迁移成功，也不能把 5.1 的测试结果沿用到 6.1。
只有 Compatibility Checker 或实际启动发现明确驱动问题时，再安排涉及全机任务的驱动维护。

6.1 增加的多物理后端与相关工具可以作为后续研究能力；本次机械臂跟随木块的原因是接触后仍用
已被机械臂改变的木块速度继续外推目标。引擎升级不会自动移除这条反馈。
