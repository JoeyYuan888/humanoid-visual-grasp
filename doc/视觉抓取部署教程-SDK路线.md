# 视觉抓取部署教程 - SDK 路线

> 文档版本：v3.0-SDK  
> 当前阶段：Step 3，等待完成头部 RealSense 的 CAM2HEAD 手眼标定  
> SDK 路线完成度：约 55%  
> 整体项目完成度：约 45%

这份文档只写 **SDK 路线**。目标是先用已验证的 `/zj_humanoid/upperlimb/*` 和 `/zj_humanoid/hand/*` 接口打通第一版真实抓取闭环。

MPC 路线单独看：[视觉抓取部署教程-MPC路线.md](视觉抓取部署教程-MPC路线.md)。

## 0. 当前结论

当前业务流程已经改为：

```text
1. 头部 RealSense 低头看桌面
2. YOLO 检测塑料袋
3. aligned depth 计算每个塑料袋在相机坐标系下的 3D 坐标
4. 通过 CAM2HEAD + HEAD2BASE 把相机坐标转成机器人可执行目标
5. SDK 路线执行：P1 -> P2 -> P3 安全路径，再小范围接近抓取
6. 抓取后把物体移动到头部相机前
7. 近距离识别二维码，并绑定到已抓取物体
8. 回收手臂：必须 P3 -> P2 -> P1，再 go_down；脖子最后 go_home
```

关键点：

- 桌面阶段不再要求识别二维码，抓前只需要塑料袋位置和深度坐标。
- 当前还不能做自动抓取闭环，因为缺少 `HEAD -> realsense_head_link` 或 `HEAD -> realsense_head_color_optical_frame` 外参。
- 手眼链路必须按课程手册来：`camera -> CAM2HEAD -> HEAD2BASE(tf) -> BASE object pose -> GRASP_OFFSET -> TCP target`。
- 不再使用之前的简化版 `camera_to_mpc` 平移拟合。

## 1. 已完成进度

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 项目目录整理 | 已完成 | 根目录只保留主入口和核心目录，测试脚本进 `tools/`，文档进 `doc/` |
| 视觉检测 | 已完成 | `models/best.pt` 可识别塑料袋 |
| 深度测距 | 已完成 | 使用 aligned depth compressedDepth，输出相机坐标系 `x/y/z`，单位 mm |
| 桌面 QR | 已取消 | 当前业务改为抓取后近距离扫码 |
| 手掌接口 | 已验证 | `joint_switch/right`、`finger_pressures/right`、`zero` 正常 |
| 右臂 SDK 路径 | 已验证 | `movej_by_path/right_arm` 可执行 P1/P2/P3 |
| 脖子低头 | 调整中 | `movej/neck joints: [0.0, 0.43]`，0.45 视角偏低 |
| 脖子复位 | 已验证 | `/zj_humanoid/upperlimb/go_home/neck` 正常 |
| 手眼标定 | 进行中 | 已保存厂商候选矩阵，但仍需本机验证 |

## 2. 当前文件位置

核心入口：

```bash
python run_grasp.py
```

常用调试工具：

```bash
python tools/debug_ros_streams.py
python tools/test_yolo.py
python tools/debug_vision_pipeline.py
python tools/debug_select_target.py
python tools/debug_sdk_interfaces.py
python tools/run_sdk_grasp_dry_run.py
```

手眼标定文件：

```text
handeye_calibration/HAND_EYE_ALIGNMENT_PLAN.md
handeye_calibration/test_cam2head_candidate.py
handeye_calibration/debug_handeye_sources.py
handeye_calibration/collect_cam2head_pairs.py
handeye_calibration/solve_cam2head.py
handeye_calibration/data/
handeye_calibration/calibration/
```

路径文件：

```text
paths/neck_look_down.json
paths/neck_home.json
paths/teach_path_right_arm.json
paths/teach_path_right_arm_return.json
```

## 3. 环境与接口检查

后续项目统一使用 `detect` 环境：

```bash
conda activate detect
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

要求：

```text
torch 版本必须带 +cu128
torch.cuda.is_available() 必须是 True
```

RTX 5060 Laptop GPU 是 `sm_120`，旧的 `+cu121` wheel 只支持到 `sm_90`，会报 `CUDA error: no kernel image is available for execution on the device`。如果 CUDA 不可用，项目会直接报错，不会退回 CPU。

先确认 ROS 图像、深度、SDK、手掌接口都在线：

```bash
python tools/debug_ros_streams.py
python tools/debug_sdk_interfaces.py
```

### 3.1 SDK 调用前必须关闭 MPC

只要接下来要调用任何 SDK 控制接口，都先执行：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
```

适用范围包括：

```text
/zj_humanoid/upperlimb/movej/*
/zj_humanoid/upperlimb/movej_by_path/*
/zj_humanoid/upperlimb/movel/*
/zj_humanoid/upperlimb/go_home/*
/zj_humanoid/upperlimb/go_down/*
/zj_humanoid/upperlimb/teach_mode/*
/zj_humanoid/hand/joint_switch/*
```

原因：SDK 和 MPC 都可能控制上肢/脖子，不能同时抢控制权。项目代码里的 `SDKMotionClient` 已经加了自动关闭 MPC 的保护，但手动在终端测试时仍然要显式先关。

手掌接口已验证：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/finger_pressures/right/zero "{}"
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/joint_switch/right "q: [-0.5, 1.2, 0.0, 0.0, 0.0, 0.0]"
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/joint_switch/right "q: [0.2, 1.2, 0.6, 0.6, 0.6, 0.6]"
rostopic echo -n 1 /zj_humanoid/hand/finger_pressures/right
```

当前实机只有：

```text
/zj_humanoid/hand/finger_pressures/right
/zj_humanoid/hand/finger_pressures/right/zero
```

课程手册里提到的右手腕力接口当前没找到，第一版 SDK 抓取不要依赖它：

```text
/zj_humanoid/hand/wrist_force_sensor/right
/zj_humanoid/hand/wrist_force_sensor/right/zero
```

## 4. 手指限位

右手 `joint_switch` 的 6 个关节含义：

```text
q[0] THUMB_MP   拇指屈曲
q[1] THUMB_CMC  拇指摆动
q[2] INDEX      食指屈曲
q[3] MIDDLE     中指屈曲
q[4] RING       无名指屈曲
q[5] LITTLE     小指屈曲
```

限位必须遵守：

| Joint | Min rad | Max rad |
| --- | ---: | ---: |
| THUMB_MP | -0.7854 | 0.7854 |
| THUMB_CMC | -0.3491 | 1.5708 |
| INDEX | 0.0 | 1.3963 |
| MIDDLE | 0.0 | 1.3963 |
| RING | 0.0 | 1.3963 |
| LITTLE | 0.0 | 1.3963 |

当前安全姿态：

```text
张开: [-0.5, 1.2, 0.0, 0.0, 0.0, 0.0]
闭合: [0.2, 1.2, 0.6, 0.6, 0.6, 0.6]
```

## 5. 头部相机姿态

桌面检测前先低头：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/movej/neck "joints: [0.0, 0.43]
v: 0.1
acc: 0.1
t: 4.0
is_async: false
arm_type: 4"
```

抓取/放置流程结束后复位脖子：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/go_home/neck
```

注意：抓取后如果还要把物体放到头部相机前扫码，脖子是否复位要按业务阶段决定。完整流程最后再复位。

## 6. 视觉检测与深度定位

主程序：

```bash
python run_grasp.py
```

视觉专项 debug：

```bash
python tools/debug_vision_pipeline.py
python tools/debug_vision_pipeline.py --no-window
```

当前配置在 `robot_grasp/config.py`：

```text
YOLO_MODEL = "models/best.pt"
YOLO_TARGET_CLASSES = []
ENABLE_DEPTH = True
DEPTH_TRANSPORT = "compressedDepth"
ENABLE_QR = False
```

视觉输出每个目标都应该包含：

```text
label / conf / bbox / valid / x_mm / y_mm / z_mm / depth_mm
```

这里的 `x_mm/y_mm/z_mm` 是相机坐标系：

```text
z: 相机前方
x: 图像右方
y: 图像下方
单位: mm
```

## 7. 手眼标定

SDK 路线也必须做手眼标定。原因是 SDK `movel` 或 TCP 目标不能直接吃相机坐标。

官方链路：

```text
camera point
  -> CAM2HEAD
  -> HEAD2BASE(tf)
  -> BASE object pose
  -> GRASP_OFFSET
  -> TCP target
```

当前确认：

- `/tf_static` 里有 RealSense 内部坐标变换。
- `/tf` 里有机器人身体链路。
- RViz 里机器人 TF 树和 RealSense TF 树断开。
- 缺少 `HEAD -> realsense_head_link` 或 `HEAD -> realsense_head_color_optical_frame`。
- `/jzhw/calib/camera/up/down` 是 docking 相机，不是当前头部 RealSense，不能直接用。

下一步优先问厂商工装输出：

```text
1. 是否直接给 HEAD -> realsense_head_link
2. 是否直接给 HEAD -> realsense_head_color_optical_frame
3. 如果不给外参，是否能输出 camera frame 点和 HEAD frame 点
4. 工装类型是棋盘格、Aruco、AprilTag，还是厂商自定义
```

如果拿到点对 CSV，用：

```bash
python handeye_calibration/solve_cam2head.py handeye_calibration/data/pairs.csv
```

当前已有厂商候选矩阵：

```text
handeye_calibration/calibration/cam2head_vendor_20260724.json
```

它数学上是正常刚体变换，但来源是别人标定结果，所以只能先 dry-run：

```bash
python handeye_calibration/test_cam2head_candidate.py --latest-csv
python handeye_calibration/test_cam2head_candidate.py --point-mm 100 150 1200
```

通过本机工装/尺子验证前，不要把这个矩阵接到真实 SDK 运动。

## 8. SDK 安全路径

桌沿风险已经确认：手臂从低位直接往目标点移动可能碰桌边。

右臂安全路径定义：

```text
P1: 起点/收回点
P2: 抬高避桌沿点，贴近身体能抬起的最高点
P3: 物体上方点
```

进场：

```text
P1 -> P2 -> P3
```

回收：

```text
P3 -> P2 -> P1
```

重要：当右臂在 P3 或桌面上方附近时，不要直接调用：

```bash
rosservice call /zj_humanoid/upperlimb/go_down/dual_arm
```

即使已经关闭 MPC，也不能从 P3 附近直接 `go_down`。必须先走 `P3 -> P2 -> P1`，确认回到安全收回点后，再按需要：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/go_down/dual_arm
```

已验证完整 P1/P2/P3 路径可执行：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/unlock
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/go_down/dual_arm
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/movej_by_path/right_arm "path:
- joint: [0.000023968450477696024, 0.0, -0.08723317551357468, 0.00005992112619424006, -0.0000001797633785827202, -0.000023968450477696024, 0.0, 0.00000005932784771706937]
- joint: [-0.26836275577352353, 1.0325009254529505, -0.3613843120774618, -0.11116567331555416, -2.0358809026290148, 0.7943144488308462, -0.34401365221653335, -0.014396198887785467]
- joint: [0.5020911006067763, -0.6137121744814067, -0.601464296287304, 0.33498306387627963, -1.0969713234689835, 0.7942665119298908, -0.09953421145923184, 0.22472297082827883]
time: 0.0
timestamp: [0.1, 12.0, 24.0]
is_async: false
arm_type: 2"
```

注意：`timestamp` 第一个值不能用 `0.0`，现场报过错；用 `0.1` 可以执行。

## 9. SDK 抓取代码框架

当前建议先做干跑：

```bash
python tools/run_sdk_grasp_dry_run.py
```

它应该只做：

```text
1. 可选低头
2. 运行视觉检测塑料袋
3. 选择一个 valid=True 的目标
4. 打印准备发送的运动目标
5. 不自动移动右臂，不自动闭手
```

手眼标定完成后，再接入真实运动：

```text
1. neck lookdown
2. run_grasp / vision_pipeline 输出 plastic bag 3D
3. camera -> CAM2HEAD -> BASE/TCP
4. movej_by_path: P1 -> P2 -> P3
5. movel 到物体上方
6. movel 慢速下降
7. hand joint_switch 闭合
8. finger_pressures 判断是否抓住
9. movej_by_path: P3 -> P2 -> P1 或移动到相机前扫码位
10. 近距离 QR
11. 安全回收，neck go_home
```

## 10. 当前不要踩的坑

- 不要把相机坐标直接发给 SDK `movel`。
- 不要再用简化版 `camera_to_mpc` 或固定 offset 代替手眼标定。
- 不要在 P3 附近直接 `go_down`，要先走 `P3 -> P2 -> P1`。
- 不要依赖未找到的右手腕力接口。
- 不要在 SDK 和 MPC 同时抢控制权时发运动命令。
- 不要在桌面阶段强依赖 QR；当前 QR 应该抓后近距离识别。
- 不要把 `/jzhw/calib/camera/up/down` 当成头部 RealSense 的外参。

## 11. 下一步

SDK 路线下一步不是继续写闭环，而是补齐手眼：

```text
1. 等厂商工装/外参回复
2. 得到 CAM2HEAD 或 HEAD -> RealSense 外参
3. 写入 handeye_calibration/calibration/
4. 用 robot_grasp/coordinate_utils.py 跑一次相机点到 BASE/TCP 的转换
5. 再恢复 SDK 干跑，确认打印目标合理
6. 最后再放开真实右臂运动
```
