# Navigation Stage

导航阶段只负责移动底盘到指定地图点，不负责抓取、搬运或放置动作。

```text
input : OCR/QR result from grasp stage
logic : destination selection, map/localization integration, navigation command
output: robot arrives at target area and reports arrival status
```

当前先实现基础导航入口，不接音频、不接障碍物语音、不接多余业务逻辑。

## 入口

```bash
python apps/navigation/run_navigation_flow.py \
  --goal demo_start
```

直接传 waypoint：

```bash
python apps/navigation/run_navigation_flow.py \
  --waypoint 0.1714 1.351 0.0 0.0 0.0 0.0 1.0
```

多航点：

```bash
python apps/navigation/run_navigation_flow.py \
  --waypoint 0.1714 1.351 0.0 0.0 0.0 0.0 1.0 \
  --waypoint -0.648 4.131 0.0 0.0 0.0 0.0 1.0
```

dry-run：

```bash
python apps/navigation/run_navigation_flow.py \
  --goal demo_start \
  --dry-run
```

## 当前接口

来自导航 demo 的有效信息：

```text
action: /zj_humanoid/navigation/navigation
type:   navigation/NavigationAction
goal:   navigation/NavigationGoal
wp:     navigation/Waypoint
odom:   /zj_humanoid/navigation/odom_info
odom type: nav_msgs/Odometry
```

waypoint 格式：

```text
x, y, z, qx, qy, qz, qw
```

速度和安全距离通过 `goal.header.stamp` 传：

```text
header.stamp.secs  = speed_cm_per_s
header.stamp.nsecs = safe_dist_cm
header.frame_id    = map
```

其他固定字段：

```text
goal.task_type.value = 0
goal.translation.enable = False
goal.translation.heading = 0.0
waypoint.distance_tolerance = 0.10
waypoint.heading_tolerance  = 0.10
```

成功状态码：

```text
NavigationState.SUCCESS = 6
```

## Odom 前置检查

发送导航目标前必须先确认定位/里程计可用：

```text
/zj_humanoid/navigation/odom_info
```

脚本默认等待 3 秒。如果没有收到 `nav_msgs/Odometry`，直接退出，不发送导航目标。

正常时终端会打印：

```text
[动作] 检查 odom: /zj_humanoid/navigation/odom_info
[✓] odom 正常: frame=map, child=body_norm, x=..., y=..., yaw=...deg
```

临时跳过检查：

```bash
python apps/navigation/run_navigation_flow.py \
  --goal demo_start \
  --skip-odom-check
```

正式运行不要跳过。

## 配置

默认配置：

```text
configs/navigation.yaml
```

默认 odom：

```text
odom_topic: /zj_humanoid/navigation/odom_info
odom_timeout_sec: 3.0
```

当前保留 demo 点位：

```text
demo_start
demo_mid
demo_end
```

后续需要现场测量并替换：

```text
grasp_area
shelf_place_area
```

## 使用边界

```text
apps/navigation/
```

不要把导航逻辑写进：

```text
apps/grasp/
apps/place/
apps/transport/
```

完整链路后续由 `apps/run_full_delivery_flow.py` 串联各阶段。

## 不迁入的 demo 功能

```text
1. audio action。
2. 错误码音频播报。
3. 障碍物语音防抖。
4. 默认自动跑多业务任务。
```

当前只保留：

```text
构造 NavigationGoal
发送 action goal
等待 result
打印 state/cause
```
