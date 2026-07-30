# MPC 路线问题记录（2026-07-21）

这份文档记录 MPC 路线当前遇到的接口问题和绕行策略。后续调试 MPC 时先看这里，避免重复排查。

## 当前 MPC 路线策略

当前先不使用：

```text
/wa/joints_seq_tracking
/DualArmMobile/currenState
```

原因是 `currenState` 的自定义消息暂时无法在当前 `rosbridge` 容器解析，无法确认 `stateTrajectory[0].value` 的维度和关节顺序。

当前优先测试：

```text
/wa/points_seq_tracking
/DualArmMobile/currentEEPose/FrameL
/DualArmMobile/currentEEPose/FrameR
```

也就是说，MPC 当前先走“任务空间末端位姿”路线，不走“关节空间路径复现”路线。

## 已确认内容

MPC 服务前缀：

```text
/wa
```

已确认服务：

```text
/wa/points_seq_tracking
/wa/joints_seq_tracking
```

已确认话题：

```text
/DualArmMobile/currentEEPose/FrameL  geometry_msgs/PoseStamped
/DualArmMobile/currentEEPose/FrameR  geometry_msgs/PoseStamped
```

左右末端位姿都能采到样本。最近一次样本见：

```text
mpc_interface_status.json
```

## 问题 1：currenState 话题类型存在，但当前容器不能解析

现象：

```bash
rostopic type /DualArmMobile/currenState
# ocs2_msgs/mpc_target_trajectories

rosmsg show ocs2_msgs/mpc_target_trajectories
# Unable to load msg ... unknown package [ocs2_msgs]

rospack find ocs2_msgs
# package 'ocs2_msgs' not found
```

判断：

```text
/DualArmMobile/currenState 已经在 ROS master 注册。
但当前 root@rosbridge:/third_party 环境没有 ocs2_msgs 消息包。
因此 rostopic echo 无法反序列化该自定义消息。
```

影响：

```text
无法确认 stateTrajectory[0].value 长度。
无法确认 WA2 当前 joints_seq_tracking 的 joint_num。
无法确认 joints_seq_tracking 所需关节顺序。
```

当前决策：

```text
暂时冻结 /wa/joints_seq_tracking。
不要发送任何 joints_seq_tracking 命令。
```

后续如果要恢复：

```bash
# 找到 MPC workspace 或运行 MPC 节点的容器
rospack find ocs2_msgs
rosmsg show ocs2_msgs/mpc_target_trajectories
rostopic echo -n 1 /DualArmMobile/currenState
```

只有能看到 `stateTrajectory[0].value` 的实际数组后，才允许继续 `joints_seq_tracking`。

## 问题 2：腕力/导纳相关话题不存在

未发现：

```text
/wrist_force_control/left_arm_compensated_force
/wrist_force_control/right_arm_compensated_force
```

影响：

```text
暂时不要测试 MPC 导纳路线。
暂时不要写依赖腕力反馈的抓取逻辑。
```

当前决策：

```text
先不启用 admittance_mode_setting。
先不调用 points_seq_tracking_with_admittance。
```

## 当前允许继续测试的内容

### 1. 只读接口确认

```bash
python tools/debug_mpc_interfaces.py
```

它只查服务、话题和末端位姿，不发运动命令。

### 2. 任务空间 points_seq_tracking 干跑计划

下一步建议写一个干跑脚本，只做：

```text
读取 currentEEPose/FrameR
生成“非常小位移”或“原地保持”的 points_seq_tracking 请求
打印请求，不发送
```

确认请求结构无误后，再考虑加一个明确的 `--execute` 开关。默认必须是 dry-run。

### 3. 第一条真实 MPC 测试原则

如果要真实发送 `points_seq_tracking`，第一条命令必须满足：

```text
只控制右臂 FrameR
目标从当前 pose 出发
位移非常小，例如 z + 0.02m 或原地保持
duration >= 5s
weight <= 1.0
type = "quintic"
手放急停
```

不要一开始就把视觉目标坐标发给 MPC。

## 当前禁止项

```text
禁止发送 /wa/joints_seq_tracking
禁止启用导纳/腕力路线
禁止直接把相机坐标发给 /wa/points_seq_tracking
禁止 SDK 和 MPC 同时抢控制权
禁止使用旧文档中的 /wa1 前缀
禁止猜测 /wa2 前缀
```

## 下一步

下一步应该做：

```text
MPC points_seq_tracking dry-run 脚本
  -> 读取 FrameR 当前位姿
  -> 构造右臂 PoseArray
  -> 打印 /wa/points_seq_tracking 请求
  -> 默认不发送
```

已创建：

```bash
python tools/run_mpc_points_dry_run.py
```

默认不发送运动命令。第一轮只检查 request 结构。

当前 dry-run 结果：

```text
已成功读取 /DualArmMobile/currentEEPose/FrameR
已构造 /wa/points_seq_tracking request
right_poses: 2 个 pose，当前 pose -> 目标 pose
left_poses: 空
time_points: [5.0, 5.0]
max_period: 12.0
weight: 1.0
type: quintic
未发送运动命令
```

完成 dry-run 后，再决定是否加 `--execute` 做 2cm 小幅真机测试。
