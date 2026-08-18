# Place App

放置阶段负责：右手持塑料袋到货架前，左手拉出目标箱，右手投放，左手推回箱子，最后双手恢复到预放置姿态。

当前完整流程已跑通，参数固定。详细背景和调试记录见 `docs/place.md`。

## 入口

二层货架完整放置：

```bash
python apps/place/run_place_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 2 \
  --execute-delay 0 \
  --execute
```

三层货架使用：

```bash
python apps/place/run_place_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 3 \
  --execute-delay 0 \
  --execute
```

## 固定参数

```text
AprilTag:
  family = tag36h11
  size   = 0.020m
  二层   = id2, neck_y=0.40
  三层   = id3, neck_y=0.25

左手:
  拉箱抓取点 offset = (-0.0819, -0.0180, 0.4883)
  拉出 20cm offset  = (-0.2819, -0.0180, 0.4883)
  推回多推 1cm      = (-0.0719, -0.0180, 0.4883)
  脱离回拉点        = (-0.0819, -0.0180, 0.4883)
  上方安全高度      = 0.12m

右手:
  投放点 offset     = (-0.1819, -0.0680, 0.6883)
  路径              = 当前右手 -> place_right_drop_mid_dual -> 投放点
  返回              = 投放点 -> place_right_drop_mid_dual -> place_ready_after_grasp_dual 右手 pose
```

## 关键 Pose

```text
data/poses/place/place_ready_after_grasp_dual.json
data/poses/place/place_left_pull_mid_dual.json
data/poses/place/place_left_pull_grasp_dual.json
data/poses/place/place_right_drop_mid_dual.json
```

## 分段调试

只跑到左手拉出 20cm：

```bash
python apps/place/run_place_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 2 \
  --execute-delay 0 \
  --stop-after-stage pulled \
  --execute
```

只跑到右手投放点：

```bash
python apps/place/run_place_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 2 \
  --execute-delay 0 \
  --stop-after-stage right-drop \
  --execute
```

可用停止点：

```text
tag-locked
mid
above-pull
hand-grasp
pull-grasp
pulled
right-drop
right-open
right-return
pushed
lifted
hand-home
ready
```

## 子入口

AprilTag 锁存：

```bash
python apps/place/run_apriltag_lock.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 2
```

左手拉箱点：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.00 \
  --execute-delay 0 \
  --execute
```

右手投放点：

```bash
python apps/place/run_right_drop_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --via-file data/poses/place/place_right_drop_mid_dual.json \
  --offset-x -0.1819 \
  --offset-y -0.0680 \
  --z-offset 0.6883 \
  --execute-delay 0 \
  --execute
```
