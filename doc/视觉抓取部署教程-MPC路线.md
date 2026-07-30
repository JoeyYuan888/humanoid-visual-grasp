# 视觉抓取部署教程 - MPC 路线

> 文档版本：v3.0-MPC  
> 当前阶段：Step 3，暂定厂商 CAM2HEAD 矩阵方向可用，等待尺子/工装确认绝对精度；正在用可修改的 `huimin1.4` 容器重建 MPC/rosbridge 环境  
> MPC 路线完成度：约 45%  
> 整体项目完成度：约 45%

这份文档只写 **MPC 路线**。目标是在完成手眼标定后，把视觉输出接到 `/wa/points_seq_tracking` 等 MPC 接口，实现更适合全身/多约束控制的抓取执行。

SDK 路线单独看：[视觉抓取部署教程-SDK路线.md](视觉抓取部署教程-SDK路线.md)。

## 0. 快速实操流程

本节记录当前已经跑通的 MPC 实机调试顺序。后续继续测试时优先按这里走。

### 0.1 终端分工

机器人/容器终端：

```text
执行 rosservice / rostopic
启动 rosbridge 9091
开关 MPC 模式
机器人 restart / 头部复位
```

本机 PC 终端：

```text
conda activate detect
运行 run_grasp.py
运行 tools/run_mpc_points_dry_run.py
运行 tools/run_mpc_visual_grasp_test.py
```

当前不要修改旧 rosbridge。MPC 测试使用我们自己的容器和 `9091`：

```text
ws://192.168.20.98:9091
```

### 0.2 启动 MPC rosbridge

在机器人自己的 `huimin1.4` 容器里：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

确认 `mpc_target` 服务类型能被 rosapi 看见：

```bash
rosservice call /rosapi/service_type "service: /wa/points_seq_tracking"
```

期望输出：

```text
type: "mpc_target/PointsSeqTracking"
```

### 0.3 机器人复位与模式切换

如果机器人状态异常，先做机器人 restart：

```bash
rosservice call /zj_humanoid/robot/set_robot_state/restart
```

当前实机验证过的完整复位流程：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/upperlimb/go_home/whole_body "arm_type: 15"
rosservice call /zj_humanoid/upperlimb/go_down/dual_arm
```

已验证输出：

```text
go_home/whole_body arm_type=15 -> success: True
go_down/dual_arm              -> success: True
```

注意：`go_home/whole_body` 不能用 `arm_type: 0`，当前机器人会返回 `The arm type is invalid for the current robot model`。实测完整复位使用 `arm_type: 15`。

SDK 类动作前先关闭 MPC，例如手爪开合、示教模式：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
```

注意：厂商已回复 **SDK 手臂路径不能和 MPC 手臂路径组合使用**。因此后续抓取路线不再使用 SDK 的 P1/P2/P3 去接 MPC。P1/P2/P3 只能作为安全路径思路参考，真正执行时要改成 MPC `PoseArray` 里的路径约束点。

MPC 手臂运动前再开启 MPC：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: true"
```

头部低头看桌面改用 MPC neck 服务，当前建议值是 `Neck_Y=0.43`：

```bash
rosservice call /wa/wa_hardware_interface/neck_movej "neck_joint: [0.0, 0.43]
t: 4"
```

脖子复位：

```bash
rosservice call /wa/wa_hardware_interface/neck_movej "neck_joint: [0.0, 0.0]
t: 4"
```

重要原则：

```text
1. 低头检测时，必须用当时的 HEAD TF 计算并锁存 BASE 目标。
2. 锁存 BASE 目标后，脖子可以复位。
3. 脖子复位后，不要用旧 CSV + 新 HEAD TF 重新计算目标。
```

### 0.4 一键低头、检测、锁存、抬头

推荐使用完整感知锁存脚本。它会：

```text
1. 调用 /wa/wa_hardware_interface/mpc_mode_setting 开启 MPC mode
2. 调用 /wa/wa_hardware_interface/neck_movej 低头到 [0.0, 0.43]
3. 读取 /zj_humanoid/upperlimb/joint_states，确认 Neck_Y 实际到位
4. 运行视觉 pipeline，检测 plastic bag 并保存 grasp_data_*.csv
5. 在低头姿态下采样 TF，转换 camera -> HEAD -> BASE
6. 保存 data/mpc_locked_target_latest.json
7. 调用 /wa/wa_hardware_interface/neck_movej 抬头到 [0.0, 0.0] 并确认 Neck_Y 复位
```

`/wa/waist_lock_setting neck_track=true` 当前不是主流程依赖。厂商确认 neck_movej 可用条件是先开启 MPC mode。

在本机 PC：

```bash
conda activate detect
python tools/run_mpc_perception_lock.py --ws-url ws://192.168.20.98:9091
```

输出会落在：

```text
data/grasp_data_*.csv
data/mpc_locked_target_latest.json
```

可用分析脚本确认视觉检测：

```bash
python tools/analyze_perf.py
```

### 0.5 手动锁存视觉目标备用流程

如果需要打开窗口手动观察，则保留旧流程：

```bash
python run_grasp.py --ws-url ws://192.168.20.98:9091
```

窗口里确认检测到塑料袋后按 `s` 保存。低头姿态不要动，再把最新 CSV 中的目标锁存成 BASE 坐标：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --save-target
```

这一步只保存目标，不发手臂运动命令。

手动流程中，目标锁存后用 MPC neck 服务抬头：

```bash
rosservice call /wa/wa_hardware_interface/neck_movej "neck_joint: [0.0, 0.0]
t: 4"
```

### 0.6 MPC 小步运动验证

先开启 MPC：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: true"
```

先验证 MPC 服务和右臂小位移：

```bash
python tools/run_mpc_points_dry_run.py --ws-url ws://192.168.20.98:9091 --dx 0.01 --execute
```

已验证结果：

```text
3cm 朝视觉目标小步成功
10cm 朝视觉目标小步成功
```

### 0.6.1 读取并保存目标手掌姿态

先在安全高位把右手调整到你希望抓塑料袋时的手掌姿态，然后读取当前 MPC 末端姿态：

```bash
python tools/capture_mpc_pose.py --ws-url ws://192.168.20.98:9091 --arm right --output data/mpc_grasp_pose_right.json
```

输出里会打印：

```text
--orientation X Y Z W
```

后续推荐直接使用文件：

```text
--orientation-file data/mpc_grasp_pose_right.json
```

### 0.6.2 设置 MPC 路径约束点

纯 MPC 路线通过 `PoseArray` 中的中间点约束路径，不再走 SDK P1/P2/P3。脚本支持反复添加：

```text
--via-point X Y Z
```

`via-point` 是 MPC/BASE 坐标，单位米。可以先 dry-run 查看 request：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --orientation-file data/mpc_grasp_pose_right.json --via-point 0.30 -0.30 0.95 --step-distance 0.05 --max-motion 0.05
```

继续朝锁存目标小步移动：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --orientation-file data/mpc_grasp_pose_right.json --step-distance 0.15 --max-motion 0.15 --execute --confirm-target
```

实测手臂至少需要上抬约 `18cm` 才能明显超过桌沿，因此当前脚本默认 `--safe-travel-z=0.95`。如果现场仍不够高，继续显式加大：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --safe-travel-z 1.00 --step-distance 0.15 --max-motion 0.15 --execute --confirm-target
```

如果 15cm 成功，再逐步增加，例如：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --step-distance 0.20 --max-motion 0.20 --execute --confirm-target
```

现阶段不要加 `--include-descend`。当前只验证到物体上方安全高度点，不做下降抓取。

### 0.7 当前绝对不要踩的坑

```text
1. 不要修改旧 rosbridge；MPC 测试用 9091。
2. 启动 rosbridge 前必须 source /workspace/catkin_ws/mpc_ws/devel/setup.bash。
3. SDK 动作前关 MPC，MPC 运动前开 MPC。
4. /wa/points_seq_tracking 左右手 poses 数量必须一致；不用的手也要用当前 pose 占位。
5. 不要用旧 CSV + 复位后的头部 TF 重新算目标，必须使用 locked target。
6. 还没验证下降抓取前，不要加 --include-descend。
7. 厂商 CAM2HEAD 暂定可用，但绝对精度还没用工装/尺子闭环验证。
8. 全身 home 使用 /zj_humanoid/upperlimb/go_home/whole_body "arm_type: 15"，不要用 arm_type: 0。
```

## 1. 当前结论

当前业务流程：

```text
1. 头部 RealSense 低头看桌面
2. YOLO 检测塑料袋
3. aligned depth 计算每个塑料袋在相机坐标系下的 3D 坐标
4. 通过 CAM2HEAD + HEAD2BASE 把相机坐标转成 BASE/TCP/MPC 目标
5. MPC 路线执行：points_seq_tracking 任务空间目标
6. 后续再补 joints_seq_tracking / 全身路径 / 导纳
7. 抓取后把物体移动到头部相机前
8. 近距离识别二维码，并绑定到已抓取物体
```

当前卡点有两个：

```text
1. CAM2HEAD 厂商候选矩阵已能通过方向一致性检查，但还没通过本机尺子/工装绝对精度验证。
2. /DualArmMobile/currenState 仍缺 ocs2_msgs 环境，暂时不依赖。
```

在完成绝对精度验证和小步 MPC 安全验证前，MPC 路线只做 dry-run 和极小位移验证，不直接发送视觉抓取运动。

## 2. 已完成进度

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 项目目录整理 | 已完成 | 主程序、工具、文档、手眼标定、模型已分目录管理 |
| 视觉检测 | 已完成 | `models/best.pt` 可识别塑料袋 |
| 深度测距 | 已完成 | 使用 aligned depth compressedDepth，输出相机坐标系 `x/y/z`，单位 mm |
| 桌面 QR | 已取消 | 当前业务改为抓取后近距离扫码 |
| MPC 服务前缀 | 已确认 | 当前实际前缀是 `/wa` |
| MPC 消息包 `mpc_target` | 迁移中 | 新容器 `huimin1.4` 中位于 `/workspace/catkin_ws/mpc_ws/src/mpc_target`，需要编译并由同一环境启动 rosbridge |
| MPC EE pose | 已确认 | `/DualArmMobile/currentEEPose/FrameL` 和 `FrameR` 有样本 |
| MPC currenState | 待确认 | 话题存在，但当前环境缺 `ocs2_msgs`，无法采样解析 |
| MPC points dry-run | 已完成 | `tools/run_mpc_points_dry_run.py` 能打印 request |
| MPC execute | 待小步验证 | `mpc_target` 包已补齐，下一步只允许零位移/极小位移测试 |
| 手眼标定 | 进行中 | 已保存厂商候选矩阵，方向一致性通过，仍需尺子/工装验证 |

## 3. 当前文件位置

MPC 调试入口：

```bash
python tools/debug_mpc_interfaces.py
python tools/run_mpc_points_dry_run.py
python tools/run_mpc_visual_grasp_test.py
```

视觉调试入口：

```bash
python run_grasp.py
python tools/debug_vision_pipeline.py
python tools/test_yolo.py
python tools/debug_select_target.py
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

MPC 问题记录：

```text
doc/MPC_ROUTE_ISSUES_20260721.md
doc/mpc_interface_status.json
```

MPC 自定义消息包：

```text
新 MPC/rosbridge 容器: huimin1.4
容器内工作空间: /workspace/catkin_ws/mpc_ws
机器人/rosbridge 环境路径:
  /workspace/catkin_ws/mpc_ws/src/mpc_target
  /workspace/catkin_ws/mpc_ws/src/ocs2_msgs
  /workspace/catkin_ws/mpc_ws/src/mpc_hardware_interface
本项目留档路径:
  mpc_target/
  ocs2_msgs/
  mpc_hardware_interface/
```

在 `huimin1.4` 容器内先确认包文件完整：

```bash
cd /workspace/catkin_ws/mpc_ws/src
ls mpc_target/package.xml mpc_target/CMakeLists.txt mpc_target/srv/PointsSeqTracking.srv
ls ocs2_msgs/package.xml ocs2_msgs/CMakeLists.txt ocs2_msgs/msg/mpc_target_trajectories.msg
ls mpc_hardware_interface/package.xml mpc_hardware_interface/CMakeLists.txt mpc_hardware_interface/srv/MPCNeckJointMove.srv
```

编译：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/catkin_ws/mpc_ws
catkin_make
source devel/setup.bash
```

确认 shell 环境可解析：

```bash
rospack find mpc_target
# /workspace/catkin_ws/mpc_ws/src/mpc_target
rospack find ocs2_msgs
# /workspace/catkin_ws/mpc_ws/src/ocs2_msgs
rospack find mpc_hardware_interface
# /workspace/catkin_ws/mpc_ws/src/mpc_hardware_interface

rossrv show mpc_target/PointsSeqTracking
rossrv show mpc_target/JointsSeqTracking
rossrv show mpc_target/PointsSeqTrackingWithJoints
rossrv show mpc_hardware_interface/MPCNeckJointMove
rosmsg show ocs2_msgs/mpc_target_trajectories
```

`mpc_target` 的 `.srv` 必须和 MPC 服务端完全一致，Response 也会参与 MD5。当前已验证/使用的接口 Response 应包含：

```text
---
bool success
string message
```

如果电脑端执行时报：

```text
client wants service /wa/points_seq_tracking... to have md5sum ...
but it has ...
```

说明 huimin1.4 里编译的 `mpc_target/srv/*.srv` 和真正 MPC 服务端的定义不一致。先按文档补齐 `string message`，再清理重编译。已经踩过的具体坑：

```text
/wa/points_seq_tracking              -> PointsSeqTracking.srv 需要 string message
/wa/points_seq_tracking_with_joints  -> PointsSeqTrackingWithJoints.srv 需要 string message
/wa/joints_seq_tracking              -> JointsSeqTracking.srv 也应补 string message，避免后续同类 MD5 问题
```

启动这个新容器里的 rosbridge。必须在同一个已 source 的 shell 中启动：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

重要：`rospack find mpc_target` 只说明**当前 shell** 能找到包。  
如果 Python 通过 rosbridge 调用 `/wa/points_seq_tracking` 仍报：

```text
Unable to load the manifest for package mpc_target
ROS path [0]=/opt/ros/noetic/share/ros
ROS path [1]=/opt/ros/noetic/share
```

说明**正在运行的 rosbridge 进程**没有 source `/workspace/catkin_ws/mpc_ws/devel/setup.bash`。这时要重启新容器里的 rosbridge，或者修改该容器的 entrypoint，让它启动前 source `mpc_ws/devel/setup.bash`。

如果不想影响旧 rosbridge，可以让新容器使用另一个端口，例如 9091：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

电脑端测试时对应传：

```bash
python tools/run_mpc_points_dry_run.py --ws-url ws://<机器人IP>:9091 --dx 0.01
```

如果调用 MPC neck 时电脑端报：

```text
Unable to load the manifest for package mpc_hardware_interface
```

说明 9091 rosbridge 的环境缺 `mpc_hardware_interface`。把本项目里的 `mpc_hardware_interface/` 或 `mpc_hardware_interface_catkin_ready_20260728.zip` 放到 `/workspace/catkin_ws/mpc_ws/src/`，重新 `catkin_make`，然后重启 9091 rosbridge。

注意：厂商文档正文曾写 `MPCNeckJointMove.srv` 的 `t` 是 `int32`，但厂商实际给出的包里是：

```srv
float64[] neck_joint
float64 t
---
bool success
string message
```

后续以厂商包为准，不能再用 `int32 t`，否则会出现 `/wa/wa_hardware_interface/neck_movej` MD5 mismatch。

如果要使用 `/DualArmMobile/currenState`、`/wa/joints_seq_tracking` 或 `/wa/points_seq_tracking_with_joints`，`ocs2_msgs` 必须也在启动 rosbridge 的同一个环境里编译并 source。否则会报：

```text
ERROR: Cannot load message class for [ocs2_msgs/mpc_target_trajectories].
```

`ocs2_msgs` 补齐后，在容器内验证：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
rosmsg show ocs2_msgs/mpc_target_trajectories
rostopic echo -n 1 /DualArmMobile/currenState
```

## 4. 共同基础：视觉检测与深度定位

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

MPC 路线使用同一套视觉输出：

```bash
python run_grasp.py
```

当前配置在 `robot_grasp/config.py`：

```text
YOLO_MODEL = "models/best.pt"
YOLO_TARGET_CLASSES = []
ENABLE_DEPTH = True
DEPTH_TRANSPORT = "compressedDepth"
ENABLE_QR = False
```

每个目标输出：

```text
label / conf / bbox / valid / x_mm / y_mm / z_mm / depth_mm
```

坐标含义：

```text
z: 相机前方
x: 图像右方
y: 图像下方
单位: mm
```

注意：这是相机坐标，不能直接发给 MPC。

## 5. 共同基础：头部与手掌辅助接口

当前 MPC 路线里，头部低头/复位改用 MPC 提供的 neck 服务；手指开合仍然使用手掌 SDK 服务。

课程 MPC 文档明确说明：`Neck Joint 并不参与 tcp 解算`。因此，头部低头/复位本身不会改变右手 TCP 的运动学解算；它主要影响相机观测和 `camera -> HEAD -> BASE` 坐标转换时使用的 HEAD 姿态。

当前脚本注意事项：

```text
run_mpc_visual_grasp_test.py 使用“CSV 里的相机点 + 当前 live TF”计算 BASE 目标。
所以检测后如果还没完成 camera -> BASE 转换，不要先复位头。
一旦 BASE/MPC target 已经算出来并固定，后续是否复位头不影响该 target 的数值。
```

当前推荐顺序：

```text
1. MPC neck 低头到 Neck_Y=0.43
2. 视觉检测并保存 CSV
3. 在头仍保持 0.43 时转换 camera -> HEAD -> BASE，并锁存 target
4. target 已固定后，MPC neck 抬头到 Neck_Y=0.0
5. 开 MPC，执行小步或路径约束测试
```

推荐直接使用完整感知锁存脚本：

```bash
conda activate detect
python tools/run_mpc_perception_lock.py --ws-url ws://192.168.20.98:9091
```

如果只单独测试 MPC neck，低头看桌面：

```bash
rosservice call /wa/wa_hardware_interface/neck_movej "neck_joint: [0.0, 0.43]
t: 4"
```

脖子抬头复位：

```bash
rosservice call /wa/wa_hardware_interface/neck_movej "neck_joint: [0.0, 0.0]
t: 4"
```

手掌安全姿态和当前塑料袋抓取参数：

```text
预张开/准备抓取: [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]
抓取闭合:       [0.3, 1.0, 0.88, 0.81, 0.35, 0.35]
放开过渡:       [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]
完全放开:       [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]
```

手指限位：

| Joint | Min rad | Max rad |
| --- | ---: | ---: |
| THUMB_MP | -0.7854 | 0.7854 |
| THUMB_CMC | -0.3491 | 1.5708 |
| INDEX | 0.0 | 1.3963 |
| MIDDLE | 0.0 | 1.3963 |
| RING | 0.0 | 1.3963 |
| LITTLE | 0.0 | 1.3963 |

手掌接口：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/finger_pressures/right/zero "{}"
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]}"
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.88, 0.81, 0.35, 0.35]}"
rostopic echo -n 1 /zj_humanoid/hand/finger_pressures/right
```

放开时也分两段，先回到预张开，再完全放开：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]}"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

## 6. 手眼标定

MPC 更需要手眼标定，因为 MPC 任务空间目标必须在正确的机器人坐标系里。

官方链路：

```text
camera point
  -> CAM2HEAD
  -> HEAD2BASE(tf)
  -> BASE object pose
  -> GRASP_OFFSET
  -> MPC target pose
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

当前有效 CAM2HEAD 矩阵：

```text
handeye_calibration/calibration/cam2head_vendor_board_20260729_164716.json
```

它来自厂家手背标定板 8 组有效样本，求解残差约 `2.6 / 4.0 mm`。旧的 `cam2head_vendor_20260724.json` 只是别人标定的候选矩阵，不再作为默认抓取矩阵。

```bash
python handeye_calibration/test_cam2head_candidate.py --latest-csv
python handeye_calibration/test_cam2head_candidate.py --point-mm 100 150 1200
```

通过本机工装/尺子验证前，不要把这个矩阵接到真实 MPC 运动。

## 7. MPC 接口确认

只查接口，不发运动：

```bash
python tools/debug_mpc_interfaces.py
```

已确认：

```text
/wa/points_seq_tracking
/wa/joints_seq_tracking
/wa/points_seq_tracking_with_joints
/DualArmMobile/currentEEPose/FrameL
/DualArmMobile/currentEEPose/FrameR
mpc_target/PointsSeqTracking
mpc_target/JointsSeqTracking
mpc_target/PointsSeqTrackingWithJoints
```

待重新验证：

```text
/DualArmMobile/currenState
```

当前问题：

```text
rostopic echo -n 1 /DualArmMobile/currenState
ERROR: Cannot load message class for [ocs2_msgs/mpc_target_trajectories].
```

说明当前环境仍缺 `ocs2_msgs`。现在已拿到 `ocs2_msgs/` 并补齐 `package.xml`、`CMakeLists.txt`；需要把它和 `mpc_target/` 一起放到 huimin1.4 的 `/workspace/catkin_ws/mpc_ws/src/` 后重新 `catkin_make`，再重启 9091 rosbridge。

验证通过后，下一阶段可以测试：

```text
/DualArmMobile/currenState
/wa/points_seq_tracking_with_joints
```

用途：在末端 pose 路径之外，再给 MPC 提供关节姿态参考，减少为了达到手掌姿态导致的身体扭曲。

当前实测 `/DualArmMobile/currenState` 已可解析，`stateTrajectory[0].value` 长度为 23，因此当前机器人按 WA2 joint 顺序处理：

```text
0  x_dir_joint
1  y_dir_joint
2  z_dir_joint
3  Pitch_Y_B
4  Pitch_Y_M
5  Waist_Z
6  Waist_Y
7  Shoulder_Z_L
8  Shoulder_Y_L
9  Shoulder_X_L
10 Elbow_Z_L
11 Elbow_Y_L
12 Wrist_Z_L
13 Wrist_Y_L
14 Wrist_X_L
15 Shoulder_Z_R
16 Shoulder_Y_R
17 Shoulder_X_R
18 Elbow_Z_R
19 Elbow_Y_R
20 Wrist_Z_R
21 Wrist_Y_R
22 Wrist_X_R
```

记录示教 via 点时要同时保存 pose 和 joints：

```bash
python tools/capture_mpc_pose.py --ws-url ws://192.168.20.98:9091 --arm right --include-joints --output data/mpc_via0_home_right.json
python tools/capture_mpc_pose.py --ws-url ws://192.168.20.98:9091 --arm right --include-joints --output data/mpc_via1_pose_right.json
python tools/capture_mpc_pose.py --ws-url ws://192.168.20.98:9091 --arm right --include-joints --output data/mpc_via2_pose_right.json
python tools/capture_mpc_pose.py --ws-url ws://192.168.20.98:9091 --arm right --include-joints --output data/mpc_via3_pose_right.json
```

执行时加 `--use-joints`，脚本会调用 `/wa/points_seq_tracking_with_joints`：

```bash
python tools/run_mpc_visual_grasp_test.py \
  --ws-url ws://192.168.20.98:9091 \
  --use-locked-target \
  --via-file data/mpc_via0_home_right.json \
  --via-file data/mpc_via1_pose_right.json \
  --via-file data/mpc_via2_pose_right.json \
  --via-file data/mpc_via3_pose_right.json \
  --stop-at-last-via \
  --no-auto-lift \
  --use-joints
```

## 8. MPC points_seq_tracking 干跑

默认只打印请求，不发运动：

```bash
python tools/run_mpc_points_dry_run.py
```

它读取当前右手末端：

```text
/DualArmMobile/currentEEPose/FrameR
```

然后构造：

```text
/wa/points_seq_tracking
```

request 结构类似：

```text
left_poses: [left_current_pose, left_current_pose]
right_poses: [right_current_pose, right_target_pose]
time_points: [5.0, 5.0]
max_period: 12.0
weight: 1.0
type: quintic
```

注意：实测 `/wa/points_seq_tracking` 要求 `left_poses.poses` 和 `right_poses.poses` 数量一致。只动右手时，左手也要填同样数量的当前位姿作为“保持不动”占位；只动左手同理。不要把未使用侧留空，否则会返回：

```text
Left and right poses size mismatch
```

只有确认 MPC 环境有 `mpc_target` 包、现场安全员准备好，并且目标是零位移或极小位移时，才允许显式执行：

```bash
python tools/run_mpc_points_dry_run.py --execute
```

历史执行曾经失败：

```text
ServiceException: Unable to load the manifest for package mpc_target.
Caused by: mpc_target
```

这个问题说明 rosbridge 进程没有加载 `mpc_target`。即使当前终端 `rospack find mpc_target` 成功，也要重启 rosbridge 并确保它启动前 source `/workspace/catkin_ws/mpc_ws/devel/setup.bash`。

## 9. MPC 路线执行策略

第一阶段只做任务空间小幅移动：

```text
1. 读取当前 FrameR pose
2. 生成一个很小的安全目标，比如 z 方向上移/保持
3. points_seq_tracking dry-run 打印 request
4. execute 只在 MPC 环境和安全条件满足后测试
```

视觉目标接 MPC 的第一版测试脚本：

```bash
python tools/run_mpc_visual_grasp_test.py
```

默认行为：

```text
1. 读取最新 data/grasp_data_*.csv 的 object_results
2. 默认选择 label=plastic bag 且 3D 有效、置信度最高的目标
3. 使用 handeye_calibration/calibration/cam2head_vendor_board_20260729_164716.json 做 camera -> HEAD
4. 从 /tf 采样 BASE -> HEAD
5. 生成 object base 和物体上方 approach target
6. 读取 /DualArmMobile/currentEEPose/FrameR 当前末端姿态
7. 默认生成当前点 -> 高位安全点 -> 目标上方高位点
8. 默认不发送运动命令
```

如果要指定某个检测框：

```bash
python tools/run_mpc_visual_grasp_test.py --target-idx 1
```

如果要用指定 CSV：

```bash
python tools/run_mpc_visual_grasp_test.py --csv data/grasp_data_20260727_133848.csv
```

第一轮真实执行只能做零位移或极小位移验证：

```bash
python tools/run_mpc_points_dry_run.py --dz 0.01 --execute
```

等确认 MPC 小步移动正常后，先在低头检测姿态下锁存一次 BASE 目标。锁存以后，即使脖子复位，也不要再用旧 CSV + 新头部 TF 重新算目标：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --save-target
```

之后的 MPC 接近测试使用锁存目标：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --step-distance 0.03
```

实机验证阶段不要一次执行完整视觉目标。先使用 `--step-distance`，让末端只朝完整视觉目标方向移动一小段：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --step-distance 0.03 --execute --confirm-target
```

该脚本默认 `--max-motion 0.03`，所以 `--step-distance` 也不能超过 3cm。只有多次小步验证方向和高度都正确后，才考虑逐步放宽 `--max-motion`。

该脚本默认不下降到预抓取高度，只移动到目标上方的 `--safe-travel-z` 高位。若要把下降点也加入路径，需要显式打开：

```bash
python tools/run_mpc_visual_grasp_test.py --use-locked-target --include-descend
```

当前脚本默认只控制末端位置，不主动改变手掌/腕部姿态。具体实现是所有目标点都复用 `/DualArmMobile/currentEEPose/FrameR` 读到的当前 orientation，所以如果当前手掌竖直于地面，运动过程中它也会尽量保持这个姿态。

但抓塑料袋时 orientation 必须改。MPC 接口本身可以控制末端姿态，因为 `left_poses/right_poses` 里的每个 `Pose` 都包含：

```text
position + orientation
```

课程示例里也给了固定姿态四元数，例如 C++ `Eigen::Quaterniond(0.707, 0, -0.707, 0)`，对应 geometry_msgs：

```text
x=0.0, y=-0.707, z=0.0, w=0.707
```

脚本已经支持：

```text
--orientation-preset doc-grasp
--orientation X Y Z W
--orientation-file data/mpc_grasp_pose_right.json
```

第一步先原地测试姿态，不移动位置：

```bash
python tools/run_mpc_points_dry_run.py --ws-url ws://192.168.20.98:9091 --orientation-preset doc-grasp --execute
```

如果姿态方向正确，再在高位小步接近时带上姿态。推荐使用现场保存的姿态文件：

```bash
python tools/run_mpc_visual_grasp_test.py --ws-url ws://192.168.20.98:9091 --use-locked-target --orientation-file data/mpc_grasp_pose_right.json --step-distance 0.05 --max-motion 0.05 --execute --confirm-target
```

如果课程示例姿态不适合，就用 `--orientation X Y Z W` 输入现场测出来的目标四元数。注意 geometry_msgs 的顺序是 `x y z w`，而 C++ Eigen 示例 `Eigen::Quaterniond(w, x, y, z)` 的顺序不同。

不要直接在完整抓取路径里突然切换手掌姿态，先做原地/小步姿态测试。

每次执行前必须确认：

```text
1. target base 的 x/y/z 在右臂可达、安全、不撞桌沿的范围内
2. 物体上方点 z 已高于桌面和物体
3. 当前末端姿态适合接近，不会用奇怪角度戳桌面
4. 手在急停上
```

暂时不要做：

```text
1. 不要用 joints_seq_tracking
2. 不要依赖 /DualArmMobile/currenState
3. 不要启用导纳/腕力路线
4. 不要把视觉相机坐标直接发给 points_seq_tracking
```

等手眼完成后，MPC 抓取链路应变成：

```text
1. neck lookdown
2. run_grasp / vision_pipeline 输出 plastic bag 3D
3. camera -> CAM2HEAD -> HEAD2BASE(tf) -> BASE target
4. 根据抓取姿态和 GRASP_OFFSET 得到 MPC target pose
5. points_seq_tracking 到物体上方
6. points_seq_tracking 慢速下降
7. hand joint_switch 闭合
8. finger_pressures 判断是否抓住
9. points_seq_tracking 或安全路径移动到扫码位
10. 近距离 QR
11. 安全回收，neck go_home
```

## 10. 当前不要踩的坑

- 不要用旧示例里的 `/wa1` 或猜测 `/wa2`，当前已确认前缀是 `/wa`。
- 不要在没采到 `/DualArmMobile/currenState` 前调用 `joints_seq_tracking`。
- 不要在 rosbridge 没有 source `/workspace/catkin_ws/mpc_ws/devel/setup.bash` 时继续 execute。
- 不要把相机坐标直接发给 MPC。
- 不要用简化版 `camera_to_mpc` 或固定 offset 代替手眼标定。
- 不要把 `/jzhw/calib/camera/up/down` 当成头部 RealSense 的外参。
- 不要启用腕力/导纳路线，当前右手腕力补偿话题没有找到。
- 不要在桌面阶段强依赖 QR；当前 QR 应该抓后近距离识别。

## 11. 下一步

MPC 路线下一步：

```text
1. 在 huimin1.4 容器编译 /workspace/catkin_ws/mpc_ws/src/mpc_target
2. 用同一个 source 过 mpc_ws/devel/setup.bash 的 shell 启动 rosbridge
3. 在电脑端运行 tools/debug_mpc_interfaces.py，确认 `mpc_target` 可通过 rosbridge 解析
4. 确认 /wa/points_seq_tracking 能 execute 一个零位移或极小位移任务
5. 用尺子/工装验证厂商 CAM2HEAD 候选矩阵的绝对误差
6. 用 robot_grasp/coordinate_utils.py 跑通 camera -> BASE/MPC target
7. 再把视觉输出接进 MPC dry-run
8. 最后才开放真实抓取运动
```
