# HANDOFF - humanoid visual grasp

本文写给完全没有上下文的新会话。项目目录：

```text
/home/hmit/naviai/center
```

GitHub 仓库：

```text
https://github.com/JoeyYuan888/humanoid-visual-grasp
```

## 1. 我们在做什么

目标是做一套人形机器人视觉抓取流程：

```text
头部 RealSense 观察桌面
-> YOLO 检测 plastic bag
-> aligned depth 取 3D 点
-> CAM2HEAD 手眼矩阵
-> HEAD->BASE TF
-> 锁存 BASE 下目标坐标
-> MPC 控制右臂走约束路径抓取
-> 手掌 joint_switch 闭合抓取
```

当前路线已经明确：**手臂和头部都走 MPC 路线**。不要再退回 SDK 头部低头。手掌开合仍然调用 `/zj_humanoid/hand/joint_switch/right`，但它只是夹爪接口，不是手臂/头部路线。

## 2. 当前关键环境

PC 侧：

```bash
cd /home/hmit/naviai/center
conda activate detect
```

机器人/容器侧当前使用自有 MPC rosbridge，端口 9091：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

PC 脚本统一连：

```text
ws://192.168.20.98:9091
```

不要修改旧 rosbridge。MPC 测试走 9091。

## 3. 已完成内容

### 3.1 视觉检测

`models/best.pt` 已经能识别 `plastic bag`。当前抓取点配置在：

```text
robot_grasp/config.py
```

当前取点策略：

```text
DEPTH_ROI: 全 bbox
GRASP_POINT_X_RATIO = 0.50
GRASP_POINT_Y_RATIO = 0.333
```

含义：抓取点画在 bbox 上方 2/3 区域的中点。用户看过窗口后认为这个点可以。

### 3.2 手眼标定

正式可用矩阵当前默认是：

```text
handeye_calibration/calibration/cam2head_vendor_board_20260729_164716.json
```

这份来自厂家手背标定板采样，残差很好：

```text
position residual mean/max: 2.6 / 4.0 mm
rot spread mean/max: 1.424 / 3.237 deg
```

另外有一份 2026-07-31 新采集候选矩阵：

```text
handeye_calibration/calibration/cam2head_vendor_new_20260731.json
```

它更接近最早厂商提供的候选矩阵，但还没有完成实物闭环验证。暂时不要自动替换默认矩阵，除非用户明确要对比测试。

### 3.3 MPC 接口

已经确认并跑通过：

```text
/wa/points_seq_tracking
/wa/points_seq_tracking_with_joints
/wa/joints_seq_tracking
/wa/wa_hardware_interface/mpc_mode_setting
/wa/wa_hardware_interface/neck_movej
```

`mpc_target`、`ocs2_msgs`、`mpc_hardware_interface` 都已放到自有 MPC workspace 编译过。注意 `MPCNeckJointMove.srv` 以厂商实际包为准，`t` 是 `float64`；如果误编成 `int32 t` 会 MD5 mismatch。

### 3.4 MPC 约束路径

已经记录并能执行 0->1->2->3 的右臂 MPC 约束路径：

```text
data/mpc_via0_home_right.json
data/mpc_via1_pose_right.json
data/mpc_via2_pose_right.json
data/mpc_via3_pose_right.json
```

路径原则：

```text
第一段：当前点 -> via0 -> via1 -> via2 -> via3
  用 /wa/points_seq_tracking_with_joints
  复现示教姿态，减少身体扭曲

第二段：via3 -> 物体上方 -> 下降抓取
  用 /wa/points_seq_tracking
  抓取阶段允许全身 MPC 自己调整
```

### 3.5 手掌抓取

手掌动作必须两步走。

抓取：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]}"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.88, 0.81, 0.35, 0.35]}"
```

放开：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.3, 1.0, 0.35, 0.35, 0.35, 0.35]}"
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

执行手掌开合或示教前，先关闭 MPC：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"
```

## 4. 最近刚改了什么

### 4.1 MPC 文档清理

已从以下 MPC 文档中删除 SDK 路线表述，头部统一写成 MPC neck：

```text
doc/视觉抓取部署教程-MPC路线.md
doc/MPC路线明日操作清单-20260728.md
```

### 4.2 `run_mpc_perception_lock.py`

当前脚本用途：一键完成低头、视觉检测、坐标锁存、抬头。

已修改：

```text
1. 删除 SDK neck fallback。
2. 头部后端只允许 mpc/manual。
3. 默认 neck-down-y 改成 0.40。
4. 增加 --show-window / --no-show-window。
5. 增加 --frame-timeout。
6. 如果视觉失败或锁存失败，除非显式 --skip-neck-home，仍会尝试 MPC neck 抬头。
7. 如果找不到 BASE -> HEAD，会打印本次采到的 TF frames 和关键路径检查。
8. 视觉 ROS 客户端清理改为后台执行，避免 roslibpy 退出卡住安全抬头。
```

语法检查已通过：

```bash
python -m py_compile tools/run_mpc_perception_lock.py
```

## 5. 当前卡在哪

最新实机现象：

```text
[mpc_mode=True] {'success': True, 'message': 'Already in running mode'}
neck_movej 命令 y=0.400
before Neck_Y=0.300
after  Neck_Y=0.455
```

说明零位被重新调整过，`0.40` 命令现在实际会到 `0.455`，偏大约 `+0.055`。下一步应测试命令 `0.35`，看实际是否接近 `0.40` 且画面能看到塑料袋。

另外完整感知脚本出现：

```text
视觉窗口黑色
检测记录: 0 条
性能采样: 0 条
没有锁存目标
```

这说明当前 9091 所连接的 ROS 环境没有给脚本提供可用 RGB 帧。不是 YOLO 没识别，而是 `rgb is None` 或 frame_count 没更新。

需要先排查相机话题是否在 9091 所在 ROS 环境里有数据。

## 6. 下一步具体操作

### 6.1 先单独测试 MPC neck 低头/抬头

PC 侧：

```bash
python tools/run_mpc_perception_lock.py \
  --ws-url ws://192.168.20.98:9091 \
  --neck-only \
  --neck-down-y 0.35 \
  --neck-time 4.0 \
  --neck-verify-tolerance 0.08
```

期望：

```text
1. 低头后实际 Neck_Y 接近 0.40。
2. 抬头复位到 Neck_Y 接近 0.0。
3. 没有卡顿。
```

如果 `0.35` 画面不够，再小步试 `0.36/0.37`。不要直接回到 `0.40`。

### 6.2 排查 9091 相机流

机器人容器里查：

```bash
rostopic list | grep realsense_head
rostopic hz /zj_humanoid/sensor/realsense_head/color/image_raw/compressed
rostopic echo -n 1 /zj_humanoid/sensor/realsense_head/color/camera_info
```

判断：

```text
如果 hz 没数据：9091 所在 ROS 环境没接到头部相机流。
如果 camera_info 没数据：视觉锁存不能计算 3D。
如果都有数据但脚本黑屏：再查 roslibpy 解码/订阅。
```

### 6.3 重新跑带窗口的完整感知锁存

CUDA 还没恢复时，用 CPU 临时测试：

```bash
python tools/run_mpc_perception_lock.py \
  --ws-url ws://192.168.20.98:9091 \
  --neck-down-y 0.35 \
  --neck-verify-tolerance 0.08 \
  --detect-seconds 12 \
  --allow-cpu-detect \
  --cpu-detect-every-n-frames 8 \
  --show-window \
  --frame-timeout 5
```

目标：

```text
1. 窗口有 RealSense 画面。
2. 能看到 plastic bag bbox。
3. CSV 有性能采样和最后 object_results。
4. 成功保存 data/mpc_locked_target_latest.json。
5. 即使失败，也必须自动抬头。
```

### 6.4 成功锁存后继续 MPC 抓取

先 dry-run 第一段 via 路径：

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

再按 dry-run 输出的累计路径长度设置 `--max-motion` 后 execute。

第二段从 via3 到物体上方/下降，先 dry-run，再 execute。抓取 offset 还没完全定死，不能直接全自动抓。

## 7. 绝对不要再踩的坑

```text
1. 不要把 SDK 头部低头加回 MPC 文档或主流程。
2. 不要在 MPC mode 未开启时调用 /wa/wa_hardware_interface/neck_movej；可能返回 accepted 但实体不动。
3. 不要把旧 rosbridge 改坏；MPC 测试用自有 9091。
4. 不要用旧 CSV + 新头部 TF 重新计算目标；低头检测后必须立刻锁存 BASE 目标。
5. 不要把相机坐标直接发给 MPC。
6. 不要把 /jzhw/calib/camera/up/down 当成头部 RealSense 外参。
7. 不要混用非 MPC 手臂轨迹和 MPC 手臂轨迹。
8. 不要在 bbox 深度取点没确认前继续硬调 TCP offset。
9. 不要用最早的手掌闭合 [0.2, 1.2, 0.6, 0.6, 0.6, 0.6]，用户实测抓不住。
10. 不要忽略无 RGB 帧问题；黑窗口时先查 ROS 相机话题，不要调 YOLO。
```

## 8. CUDA 状态

当前 CUDA/驱动坏了：

```text
torch 2.10.0+cu128
torch.cuda.is_available() == False
nvidia-smi failed: couldn't communicate with NVIDIA driver
```

临时方案是 `--allow-cpu-detect`，只用于验证流程和坐标锁存，不适合最终实时性能。

恢复 GPU 后，完整流程不要加 `--allow-cpu-detect`。

## 9. 当前未提交修改

截至本文生成时，`git status --short` 显示有这些未提交内容：

```text
M doc/MPC路线明日操作清单-20260728.md
M doc/视觉抓取部署教程-MPC路线.md
M tools/run_mpc_perception_lock.py
?? handeye_calibration/calibration/cam2head_vendor_new_20260731.json
?? HANDOFF.md
```

如果用户确认这些内容有效，建议提交并推送。
