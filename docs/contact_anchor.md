# 接触后自我追逐修复（2026-10-25）

机械臂被木块带走的观感来自控制目标确实在接触后不断向前移动。根因位于 close_gripper，
不是上一轮只分析的 lift。当前专家在首次夹指接触时锁存机械臂世界坐标参考，稳定双指接触后竖直抬升。
本轮按用户选择保留 Isaac Sim 5.1 执行，仅另做 6.1 升级评估，见 isaacsim61_assessment.md。

## 完整时序揭示的问题

上一版实测代码 5706912201ed933092a53db16931321a2be2faa8，seed42：

- tick60 已有双指约 15 N 接触，木块实际 x 速度已经由带速 .03 m/s 增至 .138 m/s。
- close_gripper 仍使用 cube_xy + clip(cube_velocity_xy, -.1, .1) × .1 s 的拦截点，并等待 close_ticks=18。
- 后续机械臂的动作改变木块运动，再被当作下一次外推速度；tick62 木块速度约 .209 m/s。
- tick60 到 tick76（首次 lift）间，木块 x 增加 122.32 mm，EEF x 增加 127.50 mm，耗时 .533 s。
- 从首次接触到 episode 尾帧，EEF 最大水平偏移达到 129.14 mm，而夹持相对位置仍可能很稳定。

因此夹持没有明显摩擦滑移，并不代表控制轨迹合理。上一轮 13/13 验收只证明当时的输送、抬升和时间判据通过，
漏掉了接触后横向自我追逐。此前 -0.0694 到 -0.0631 的净位移仍是约 6.31 mm，但它选的是另一个时间区间，
不能用于否定接触阶段的厘米级异常。

## 修订后的动作生成

默认 lift_xy_mode=contact_anchor，原 fixed/track_cube 保留为诊断模式。

1. 未接触时仍基于当前方块位置和速度拦截、下降、闭爪。
2. 任一夹指与方块接触力超过 1 N 时，记录当前实际 link6 世界位置和姿态，进入 grasp_settle。
   从这一刻起，方块位置/速度不再改变机械臂的水平参考。
3. 保持该参考，连续三个控制观测都有双指接触后进入 lift。按 30 Hz 采样，首次到第三次观测相隔 2/30 s。
4. 抬升距离由此时方块高度和目标高度计算一次，Z 命令以 .25 m/s 上限逐 tick 增加；XY 和姿态固定在接触参考。
   使用独立命令参考，避免把实际末端跟踪误差逐步积分进目标。
5. 双指接触连续丢失三个 tick 则失败；稳定接触迟迟无法建立也超时失败。

这里锁存的是伺服目标，不是锁定物理状态。机械臂仍经 IK、有限关节驱动和 300 Hz PhysX 执行；
传送带继续保持 .03 m/s 接触表面速度，方块没有 attach/焊接/运行期 pose 或 velocity 写入。
原相机、机械臂基座与 Room/Background/Light 配置继续使用。

## 新的验收边界

保留原有成功条件：同一方块底面离带面超过 5 cm，且双指接触连续保持 15 tick。
另外从首次接触到尾帧统计水平偏移，默认要求：

- EEF 水平命令最大偏移 < 1 微米（应为零，用容差处理浮点）；
- 实际 EEF 最大水平偏移 ≤ 10 mm；
- 方块最大水平偏移 ≤ 20 mm，包含夹指收拢与有限接触动力学。

通过抬升条件但违反横移判据的轨迹会标记失败，不进入成功数据导出。
contact_motion 记录首次接触 tick、lift 起点、接触到 lift 的时间、抬升前位移以及全接触区间峰值。
分析包含 N+1 的尾观测。旧实验原始文件和原标签保留，作为历史判据下的数据；不与本次成功数据混用。

## 实测与复现

原装 USD 十项集成验收全部通过：0/.03/.06 输送、静态抓取、三个动态 seed 和三种模拟延迟。
36 项 CPU 测试在本地、服务器与 CI 通过，十条轨迹的严格回读及头部视频帧数核对通过。

运行代码 SHA：6ba165c86e0e05f552bc3764e39398ff7f871564。
本次 GPU0 在启动前显存占用 265 MiB，为四卡最少；用于渲染，物理后端仍为 CPU PhysX。

| 动态 seed | 旧版接触后 EEF 最大水平偏移 | 修订版 EEF 最大水平偏移 | 修订版方块最大水平偏移 | 抓取 |
| --- | --- | --- | --- | --- |
| 42 | 129.14 mm | 0.342 mm | 7.613 mm | 成功，82 tick |
| 43 | 131.20 mm | 0.353 mm | 7.731 mm | 成功，81 tick |
| 44 | 130.85 mm | 0.349 mm | 7.847 mm | 成功，81 tick |

原装四条抓取轨迹均无控制/IK 错误。默认 primitives 皮带另外通过 seed42 抓取（82 tick），
接触后 EEF 水平峰值偏移 0.342 mm，原始数据和视频回读通过；该条不并入原装 USD 的 3/3 统计。

三个 seed 的水平命令偏移均为 0。首次接触到 lift 从 .533 s 降为 .0667 s。
上述方块位移属于世界坐标变化，不能全部称作夹持摩擦滑移。三次指定 seed 成功不代表广泛分布可靠性。

```bash
cd /home/ubuntu/Workspace/dynamicbench
source /home/ubuntu/miniconda3/bin/activate RoboDojo
nvidia-smi --query-gpu=index,memory.used --format=csv
unset CUDA_VISIBLE_DEVICES
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
# 按当前占用选择 GPU；以下为本次实际使用的 GPU0。
bash scripts/dynamicbench/run_isaac.sh validate_mvp --headless --device cuda:0 --render-gpu 0 --belt-source robodojo --num-success 3 --max-attempts 3 --ticks 150 --seed 42 --output outputs/contact_anchor_repro --kit_args="--/validate/iommu/enabled=false --/validate/p2p/enabled=false --/plugins/carb.tasking.plugin/threadCount=8"
python scripts/dynamicbench/inspect_dataset.py outputs/contact_anchor_repro
python -m pytest dynamicbench/tests -q
```

完整原始结果位于服务器 /home/ubuntu/Workspace/dynamicbench/outputs/contact_anchor_20261025。
本轮尚未运行 6.1、真机、真实模型异步推理、.06 m/s 抓取或原有 42 个任务的完整回归。
