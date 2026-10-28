# 图1与 RoboDojo 官方传送带画面的差异排查（2026-10-28）

结论：**机械臂底座和主相机配置复用了官方配置；传送带布局和机械臂关节姿态没有保持一致。**
此前“复用 RoboDojo 位置”的说法范围过宽：复用资产文件、底座与相机配置，不等于复现整个官方场景。
当前 `_spawn_original_belt()` 会缩小并重新放置原装 USD，这是画面差异的主要原因。

## 直接量测与源码证据

基线为生产代码 c61f3dc；诊断脚本提交为
`d95caa6e85cc7ce13756ac46d3a2082b5cc9d1b9`。
官方参考为同一上游快照中的 `pick_from_conveyor_by_image` 任务，以及下载资产中实际保存的
`Assets/Eval_Layout/RoboDojo/arx_x5/0/pick_from_conveyor_by_image_0.json`。
图2中的展示板和篮子与该任务相符，但截图未包含视频地址、seed、时刻和版本日志，
因此参考的是这一官方实现及其保存布局，不能声称锁定了图2的原始帧。

| 项目 | 当前任务 | 官方实现/保存布局 | 判断 |
| --- | --- | --- | --- |
| 左 X5 底座位置（m） | `[-0.3,-0.45,0.765]` | 相同 | 一致 |
| 左 X5 底座方向 | 绕 Z 轴 90°，WXYZ 归一化四元数 | `[0.707,0,0,0.707]`，等价方向 | 一致 |
| 机器人数量 | 只实例化左臂 | `dual_x5.yml` 包含左右两臂 | 不一致 |
| 主相机位置（m）/欧拉角（°） | `[0,-0.41,1.308]` / `[30,0,0]` | 相同配置 | 一致 |
| 名义镜头配置 | 640×480，f=10，水平/垂直孔径22.212/14.266 | 相同 Gemini_345Lg 模板 | 一致 |
| 六个机械臂初始关节角（rad） | `[-0.14888,1.8515,1.9196,-1.6389,0,-0.14887]` | 上游默认 `[0,0,0,0,0,0]` | 不一致 |
| 传送带整机包围盒尺寸（m） | `0.700009 × 0.280009 × 0.184876` | `3.962949 × 1.176837 × 0.924380` | 明显缩小、非等比 |
| 传送带整机包围盒中心 XY（m） | `[-0.120004,-0.020006]` | `[-0.098527,0.187926]` | 不一致 |
| 传送带整机最低点 Z（m） | `0.621400` | `0.050000` | 缩小支腿后整体抬高 |
| Belt 网格最高点 Z（m） | `0.763842` | `0.762212` | 带面高度接近，主要差异在尺寸/平面布局 |

尺寸为 USD `BBoxCache` 的世界坐标轴对齐包围盒；整机范围包括支架/边框。
“Belt 网格最高点”是该网格的视觉几何量测，不能直接充当独立的碰撞支撑高度标定。
模型根节点的坐标原点不在几何中心，因此不能把下面两组根节点平移直接解释成带面中心位移。

关键代码路径：

- `dynamicbench/layout.py:18` 读取官方 `dual_x5.yml` 的左臂底座，读取 `camera_config.yml` 和镜头模板。
- `dynamicbench/envs/conveyor_cube_task.py:112` 用任务专用 `initial_q` 覆盖官方默认关节角；底座未移走，抬起/弯折的关节姿态使机械臂出现在画面左上方。
- `dynamicbench/envs/conveyor_cube_task.py:280` 重新计算传送带 XY 缩放，并把 Z 缩放固定为 `0.2`；283–287 行再重算平移。
- 当前原装 USD 根节点 scale 为 `[0.17663832,0.23793338,0.2]`、平移为 `[0.23,-0.01693066,0.6214]`、方向为单位四元数。
- 官方保存布局根节点 scale 为 `[1,1,1]`、平移为 `[-2.08,0.175,0.05]`、WXYZ 方向为 `[0,0,0,1]`（绕 Z 轴 180°）。

这会让整机长度缩到约 1/5.66、宽度缩到约 1/4.20、高度缩到 1/5；支腿缩短后为了保持带面高度又把整机抬高。
结果就是图1的“小传送带完整露出、大面积地板可见、支架没有落地”的外观。
官方原始尺寸下，同一台相机只能看到大传送带的一段，带面自然铺满画面。

相机还核对了坐标约定：官方父 Xform 使用配置姿态，pinhole 子相机为 USD 单位朝向；
当前 Lab `convention="opengl"` 与该光轴/上方向约定相符，未发现由此产生的 90°/180°相机误转。
旧 SDK 镜头 setter 会把焦距与孔径一起转换为 10 倍 USD 数值；比例不变，不能仅凭 USD 原始值不同认定视场不同。
官方 CameraManager 的配置内参接口与当前 Lab 回报均使用 `fx=fy≈288.1325, cx=320, cy=240`。
这不构成对图2未知录制环境的逐像素投影验证。

## 同一相机的三组实际渲染

三个独立进程都使用 Isaac Sim 6.1.0.0、Lab v3.0.0-EA、GPU 2。
保持同一 Room/HDR/灯光配置、主相机世界变换、镜头参数及回报内参；实际审计 JSON 已逐项比较相等。
每次在初始状态沉降后抓取图像，没有执行专家策略，捕获前后实际物理步数相等。三个进程退出码均为 0。

1. **current**：原生产场景；重新渲染出图1同类的小传送带和左上方机械臂构图。
2. **official_belt**：只恢复保存布局中的传送带根节点变换；机械臂关节不变，带面立即铺满画面。
3. **official_belt_home**：在第二组基础上恢复上游六关节默认零位；夹爪回到画面下侧，主体构图进一步接近图2。

第三组仍只有一只机械臂和一个方块，未复现完整官方任务；图2来自任务执行过程，也不能把其实际关节角当成默认零位。
这些对照确认了几何布局与关节姿态的因果影响，不是原官方视频回放，也没有对改大后的场景声称抓取成功。

本地证据目录：
`/Users/riri/Documents/Codex/2026-10-23/files-mentioned-by-the-user-dynamicbench/outputs/visual_layout_audit_20261028/`

- `controlled_comparison.png`：三组对照并排图。
- `{current,official_belt,official_belt_home}/{head.png,wrist.png,audit.json,run.log,exit_status.json}`。
- `reference_layout_0.json`、`reference_belt_metadata.json`：实际读取的官方布局/资产元数据副本。
- `comparison_metrics.json`；Git 中对应 `results/visual_layout_audit.json`。

服务器证据目录：`/home/ubuntu/Workspace/dynamicbench/outputs/visual_layout_audit_20261028`。

复现任一变体（在服务器仓库根目录，GPU 先按当前空闲显存选择）：

```bash
export OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=8
export OMNI_KIT_ACCEPT_EULA=YES
export LD_LIBRARY_PATH="$PWD/outputs/deps/root/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
outputs/envs/dynamicbench61/bin/python -u scripts/dynamicbench/audit_visual_layout.py \
  --headless --device cuda:2 --render-gpu 2 --visual-variant official_belt_home \
  --output outputs/visual_layout_repro \
  --kit_args="--/plugins/carb.tasking.plugin/threadCount=8"
```

## 其余视觉差异与修正顺序

- 图2是双臂、篮子、展示板和目标物的完整任务；当前简化任务没有右臂、篮子、展示板和相机支架。遮挡与反射对象不同。
- 当前复用 Room/HDR，但把自建物理 Ground 隐藏，显示 Room 的地板；官方场景另有可见的 `material_0564` MDL Ground。复用 Room 并不代表复用了整个官方场景。
- 核对原房间 RectLight：intensity=15000、exposure=0、normalize=false；当前重建灯保留强度及世界尺度约 2.5×0.25 m，HDR 强度同为1000。没有证据表明主体构图差异来自简单的灯强值改动。
- 官方上游默认包含 quality/DLAA、反射与 GI 设置；当前 6.1 使用原生 RTX、DLAA 和去噪，早期 5.1 任务还曾覆盖为 TAA。截图本身不能确定具体引擎/后处理版本；亮度、反射、纹理锐度的剩余差异尚未做逐参数光度对照，不能全部归因于6.1。

若要对齐官方外观，应先恢复原装 USD 的完整根节点变换及比例，再统一机器人数量/关节状态和可见 Ground/道具，最后比较相机与渲染参数。
**不能只调大 `belt.size`**：当前代码仍会固定 Z scale=0.2 并重新摆放整机。
恢复原始布局后需要重新验证实际支撑高度、机器人可达工作区及专家初始/截取目标；不能沿用缩小场景的既有抓取成功结论。

本轮新增的脚本与报告用于诊断，生产任务配置及历史示教数据均未改写。
