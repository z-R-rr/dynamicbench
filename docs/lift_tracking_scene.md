# Lift 跟踪、夹持坐标系与 RoboDojo 场景（2026-10-25）

## 从原始数据校正问题定义

用户给出的坐标对应旧 primitive 场景 seed44（代码 c1e4faf2）：tick75 的 cube_x 为 -0.0694029257 m，
tick94 为 -0.0630934089 m，净位移为 **0.0063095167 m = 6.31 mm**。该区间 x 方向累计路程为 39.66 mm，
也不是 11.5 cm。净位移、累计路程、三维路径长度不能混用。

同段双指接触力接近 15 N。tick76 时方块底面已高于带面约 10.42 mm。
SurfaceVelocityAPI 通过接触修改回调影响接触求解；它不是对已脱离皮带的物体施加的持续外力。
因此 .03 m/s × 时间不是被夹起方块必须遵守的位移。
参见 [NVIDIA PhysX Schema](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/schemas/physxschema.html)。

本任务关闭原资产运动图，并在每个物理步写入世界坐标系 [.03,0,0] m/s；新轨迹直接保存实际读取的
oracle_debug.belt_surface_velocity。原 USD 文件里 authored -0.1 的值不等于运行时生效值。

以 EEF 根坐标系测量，旧 seed44 在 lift 中有约 12.28 mm 峰值变化。但 ARX 左右夹指是独立移动关节，
实际夹持中心会随 (q7-q8)/2 改变。用真实 qpos 和同源 URDF 重建夹持中心后，方块相对该中心的峰值变化约
6.7 微米。这是仿真数值层面的运动估计，不是实际机器人计量精度。不能把全部 EEF 相对运动都称为摩擦滑移。
新轨迹直接读取左右夹指刚体位置，记录 finger_midpoint_in_eef 和 cube_in_pinch_frame，减少这种歧义。

## 对方案 A 做了实际对照

直接把固定 grasp_xy 换成当前 cube[:2]，同时继续用固定 link6→TCP 偏移，会把夹指中心偏移也当成目标运动追逐。
实测原始方案 A（28331bc）在 seed42、45 保持双指接触却持续搬运偏移，最终 out_of_reach：
这次 42..46 的五次尝试只有三次成功；预先配对的 42/43/44 为 2/3。失败数据原样保留，不计为成功示教。
同一房间、同一皮带下，fixed 的 42/43/44 均成功。

当前默认仍以 **每个控制 tick 观测到的当前方块 XY** 为 lift 目标，但使用实际夹持中心换算 TCP：

- 在双指接触建立时，记录方块相对实际双指中点的抓取偏移。
- lift 每 tick 读取左右夹指的真实位置，更新该抓取点的世界位置与 EEF→TCP 偏移。
- 对目标增量施加原有 .25 m/s 限速，再转成绝对 link6 EEF10 目标。
- 继续通过 IK、有限关节驱动和每 tick 十次物理步执行。皮带持续运行，成功仍要求底面抬升 >5 cm 且双指接触连续保持 15 tick。

记 p 为当前方块世界位置，x 和 R 为实际 link6 位置与旋转，f 为左右夹指实际位置的中点。
在建立抓取的时刻 g，记录 r = R_g^T (p_g-f_g)。运行时估计抓取 TCP 为 T = f + R r。
其目标是 [p_x,p_y,belt_top+lift_height]。限速后得到 TCP 指令 u，再以当前夹持偏移
 o = R^T(f-x)+r 换算 EEF 位置指令 x_cmd = u - R_goal o。
这些量只用于专家目标计算；方块状态仍完全来自 PhysX 接触和重力。

配置/消融：

```yaml
expert:
  lift_xy_mode: track_cube
  lift_tcp_frame: actual_pinch
scene:
  source: robodojo
  config: env_cfg/scene/conveyor.yml
  light_intensity_scale: 1.0
```

- `--lift-xy-mode fixed`：旧固定 XY 基线。
- `--lift-xy-mode track_cube --lift-tcp-frame nominal`：原始方案 A，保留用于复现失败。
- `--lift-xy-mode track_cube --lift-tcp-frame actual_pinch`：当前默认。

## 场景组件复用

读取原 env_cfg/scene/conveyor.yml，直接实例化 RoboDojo 的 Room、Background、Light：

- Room：Simple_Room/base0.usd，使用原位置、姿态和 .5 缩放。
- Background：brown_photostudio_02_4k.hdr，原 intensity=1000。
- Light：通过原 Light 类重建房间 authored RectLight 的合成位置、朝向、尺寸、颜色和强度；固定取值范围，避免引入随机化差异。
- 房间的 52 个碰撞形状仅作视觉背景；旧物理地面保留但不出现在 RGB 中。皮带、方块、机器人碰撞条件保持一致。

场景审计记录在 asset_audit.json，包含来源配置 hash、原灯光 prim 和解析后的具体参数。
`--scene-source minimal` 可切回旧简化渲染；头部相机和机器人基座继续直接读取原 RoboDojo 配置。

## 复现

先检查 GPU 占用，下例使用本次实测的 GPU1：

```bash
cd /home/ubuntu/Workspace/dynamicbench
source /home/ubuntu/miniconda3/bin/activate RoboDojo
unset CUDA_VISIBLE_DEVICES
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
bash scripts/dynamicbench/run_isaac.sh validate_mvp --headless --device cuda:1 --belt-source robodojo --compare-lift --num-success 3 --max-attempts 3 --ticks 150 --seed 42 --output outputs/lift_pinch_repro --kit_args="--/validate/iommu/enabled=false --/validate/p2p/enabled=false --/plugins/carb.tasking.plugin/threadCount=8"
python scripts/dynamicbench/inspect_dataset.py outputs/lift_pinch_repro
python -m pytest dynamicbench/tests -q
```

对照数据应按 source_commit、lift_xy_mode、lift_tcp_frame 和 scene 配置分开分析。
旧数据、原始方案 A 的失败数据和修订后的成功数据不会混合重标注。


## 正式修订版实测结果

物理运行 SHA：`5706912201ed933092a53db16931321a2be2faa8`。
原装 USD + 原生 Room/Background/Light，GPU1 渲染、CPU PhysX。
核心十项验收加三个 fixed 对照共 **13/13 通过**，33 项 CPU 测试在本地、服务器和 CI 通过。
13 个 episode 的严格回读及头部视频帧数检查通过；新的实际 surface velocity 读取值始终符合设置速度。

| 配对 seed | fixed | 直接方案 A（nominal TCP） | 实际夹持中心换算 | 修订版 pinch 相对位置峰值变化 |
| --- | --- | --- | --- | --- |
| 42 | 成功，96 tick | 失败，348 tick，out_of_reach | 成功，101 tick | 12.66 μm |
| 43 | 成功，95 tick | 成功，95 tick | 成功，95 tick | 9.18 μm |
| 44 | 成功，95 tick | 成功，95 tick | 成功，95 tick | 7.58 μm |

fixed 同一指标也约为 8.0–12.5 μm，没有证据表明修订版降低了原本已很小的摩擦滑移。
修订版的价值是实现每 tick 的当前方块 XY 跟踪，并避免用 nominal TCP 误追逐夹指中心漂移。
三 seed 只是 MVP 验证，不推断广泛分布的可靠性。原始方案 A 的五次尝试、两次失败完整保留。

本次完整数据：`/home/ubuntu/Workspace/dynamicbench/outputs/lift_pinch_room_20261025`。
原始方案 A：`/home/ubuntu/Workspace/dynamicbench/outputs/lift_tracking_room_20261025`。
小型实测记录分别在 results/lift_tracking_acceptance.json、lift_nominal_diagnostic.json、lift_comparison.json。
