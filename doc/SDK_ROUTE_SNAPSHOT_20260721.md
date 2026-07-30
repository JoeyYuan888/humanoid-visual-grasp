# SDK 路线进度快照（2026-07-21）

这份文档用于在后续转向 MPC 之前，保存当前 SDK 抓取路线的状态。SDK 相关代码、路径和接口先保留，不删除；之后如果 MPC 路线遇到问题，可以从这里恢复 SDK 方案。

## 当前任务目标

项目当前目标是：

```text
桌面阶段：头部 RealSense 检测塑料袋位置，输出相机坐标系下 3D 坐标
抓取阶段：根据 3D 坐标抓取塑料袋
识别阶段：抓起后移动到相机前近距离识别二维码
绑定阶段：把二维码文本绑定到已抓取物体
```

重要业务变化：

- 当前不再用杯子/瓶子作为最终目标，最终目标是塑料袋。
- 桌面阶段不强制二维码识别，二维码放到抓取后近距离识别。
- SDK 方案只是当前留档，下一阶段会尝试 MPC。

## SDK 路线当前完成情况

### 已实机确认的接口

上肢 SDK：

```text
/zj_humanoid/upperlimb/movej_by_path/right_arm
/zj_humanoid/upperlimb/movel/right_arm
/zj_humanoid/upperlimb/movej/neck
/zj_humanoid/upperlimb/go_home/neck
/zj_humanoid/upperlimb/unlock
/zj_humanoid/upperlimb/go_down/dual_arm
/zj_humanoid/upperlimb/stop
```

手掌：

```text
/zj_humanoid/hand/joint_switch/right
/zj_humanoid/hand/finger_pressures/right
/zj_humanoid/hand/finger_pressures/right/zero
```

当前实机没有发现这些腕力接口，第一版 SDK 方案不要依赖它们：

```text
/zj_humanoid/hand/wrist_force_sensor/right
/zj_humanoid/hand/wrist_force_sensor/right/zero
```

### 已实机确认的头部姿态

低头看桌面：

```text
paths/neck_look_down.json
service: /zj_humanoid/upperlimb/movej/neck
commanded_joint: [0.0, 0.45]
```

用户确认 `0.45` 视角刚好。

脖子复位：

```text
paths/neck_home.json
service: /zj_humanoid/upperlimb/go_home/neck
```

用户实测：

```bash
rosservice call /zj_humanoid/upperlimb/go_home/neck
success: True
message: "DualArmNeckRobotInterface Service | neck_gohome success"
```

### 已实机确认的右臂路径

右臂接近路径：

```text
paths/teach_path_right_arm.json
P1 -> P2 -> P3
```

右臂回收路径：

```text
paths/teach_path_right_arm_return.json
P3 -> P2 -> P1
```

关键点定义：

```text
P1: go_down 后的默认起点/收纳位
P2: 贴近身体的最高避桌沿点
P3: 桌面上方安全预抓取位
```

已验证可执行的 `movej_by_path` 调用方式：

```text
time: 0.0
timestamp: [0.1, 12.0, 24.0]
is_async: false
arm_type: 2
```

注意：`timestamp` 第一个值不要用 `0.0`，实机会报错。之前试过 `time: 24.0 + timestamp: []`，可能返回 success 但手臂不动，不要再用这个方式。

### 已加入但尚未实机跑完的 SDK 代码

```text
robot_grasp/sdk_motion_client.py
run_sdk_grasp_dry_run.py
```

`run_sdk_grasp_dry_run.py` 的设计目标：

```text
一键低头
  -> 跑 VisionPipeline 检测塑料袋
  -> select_grasp_target() 选 valid 目标
  -> 打印准备发送的 SDK 运动计划
  -> 不移动右臂
  -> 不闭合手掌
```

运行方式：

```bash
python tools/run_sdk_grasp_dry_run.py
```

如果脖子已经低头，只调视觉：

```bash
python tools/run_sdk_grasp_dry_run.py --skip-neck
```

只看终端输出：

```bash
python tools/run_sdk_grasp_dry_run.py --no-window
```

当前状态：代码已通过 `py_compile` 静态检查，但用户还没有在实机上跑这个步骤。

## 绝对不要再踩的坑

1. 不要从 P3 或桌面上方直接调用：

```bash
rosservice call /zj_humanoid/upperlimb/go_down/dual_arm
```

这样可能碰桌沿。必须先走：

```text
P3 -> P2 -> P1
```

2. 不要在手臂还在 P3 或桌面附近时先复位脖子。推荐顺序：

```text
右臂 P3 -> P2 -> P1
  -> /zj_humanoid/upperlimb/go_home/neck
```

3. 不要把视觉输出的相机坐标直接发给 `movel`。当前 `x_mm/y_mm/z_mm` 是 RealSense 相机坐标系，还没有完成：

```text
camera -> head -> base/TCP
```

的标定转换。

4. SDK 和 MPC 不要同时抢控制权。尝试 MPC 前，如果要动 SDK，先确保 MPC 没有在控制上肢；尝试 MPC 时也不要同时运行 SDK 运动脚本。

5. 手指闭合必须遵守限位，不要直接打满。

手指 q 顺序：

```text
[THUMB_MP, THUMB_CMC, INDEX, MIDDLE, RING, LITTLE]
```

限位：

```text
THUMB_MP:  [-0.7854, 0.7854]
THUMB_CMC: [-0.3491, 1.5708]
INDEX:     [0.0, 1.3963]
MIDDLE:    [0.0, 1.3963]
RING:      [0.0, 1.3963]
LITTLE:    [0.0, 1.3963]
```

## SDK 路线如果之后继续，下一步是什么

如果以后回到 SDK，建议从这里继续：

1. 跑：

```bash
python tools/run_sdk_grasp_dry_run.py --no-window
```

确认它能低头、检测塑料袋、选出 valid 目标并打印运动计划。

2. 建立坐标转换：

```text
camera xyz(mm) -> SDK TCP/base pose
```

这一步没完成前，不允许把视觉坐标发给 `movel`。

3. 先只启用 `movej_by_path` 到 P3，不做下探、不闭手。

4. 再启用短距离 `movel` 下探。

5. 最后启用 `hand/joint_switch/right` 闭合和 `finger_pressures/right` 判断是否抓住。

## 和 MPC 路线的关系

MPC 后续可以复用当前视觉部分：

```text
VisionPipeline
  -> object_results
  -> select_grasp_target()
```

MPC 要替换的是运动执行层：

```text
SDK: movej_by_path + movel
MPC: joints_seq_tracking + points_seq_tracking
```

所以 SDK 留档时，不需要删除视觉代码；MPC 只需要接入同一份 `object_results` 和目标选择逻辑。
