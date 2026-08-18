# Place Stage

放置阶段负责在机器人到达货架后，把右手抓取的塑料袋放入目标箱子。当前导航步骤先不接入：抓取完成后由人工移动机器人到放置点，再开始放置阶段调试。

正式路线优先使用贴在箱子把手处的 AprilTag 辅助定位；原来的蓝色/白色箱体轮廓识别保留为备选。

## 阶段边界

```text
input : 右手已抓住塑料袋、机器人到达货架放置点、头部相机 RGB/depth、箱子正面 AprilTag
logic : AprilTag 定位、Tag -> BASE 转换、左手拉箱、右手投放、推回箱子
output: 塑料袋放入箱子，箱子推回，双臂回安全姿态
```

## AprilTag 参数

```text
Tag family : AprilTag 36h11
Tag size   : 20 mm = 0.020 m
三层 Tag id: 3, neck_y=0.25
二层 Tag id: 2, neck_y=0.40
```

现场要求：

```text
标签贴在目标箱子正面把手附近。
三层用 id=3，二层用 id=2；同一层画面内不要出现重复 id。
调试图底部会标注 `shelf / neck_y / target_ids`，避免混用 0.25 和 0.40 的旧图。
低头识别前先确认右手塑料袋不会挡住标签。
```

## 当前规划流程

当前放置流程已完整跑通，参数固定。正常入口：

```bash
python apps/place/run_place_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 2 \
  --execute-delay 0 \
  --execute
```

默认固定参数来自 `configs/place.yaml` 和以下 pose 文件：

```text
data/poses/place/place_ready_after_grasp_dual.json
data/poses/place/place_left_pull_mid_dual.json
data/poses/place/place_left_pull_grasp_dual.json
data/poses/place/place_right_drop_mid_dual.json
```

```text
右手持塑料袋，人工移动机器人到货架放置点
-> MPC running mode
-> 低头
-> 头部相机识别 AprilTag 36h11，三层 id=3 / 二层 id=2
-> Tag camera pose -> HEAD -> BASE
-> 保存 BASE 锁存结果
-> 抬头
-> 左手到拉箱预抓取点
-> 左手接近箱子前沿/把手
-> 左手向身体侧拉出箱子
-> 右手经 place_right_drop_mid_dual 到投放点
-> 右手松开塑料袋
-> 右手经 place_right_drop_mid_dual 原路返回预放置姿态
-> 左手推回箱子并多推 1cm
-> 左手回拉 1cm、上抬 12cm
-> 左手手指 home
-> 左手经 place_left_pull_mid_dual 返回预放置姿态
```

除非重新贴 AprilTag、换箱子结构、重录示教点或现场碰撞风险改变，否则不要调整以下核心参数：

```text
左手拉箱抓取点 offset : x=-0.0819, y=-0.0180, z=0.4883
左手拉出点 offset     : x=-0.2819, y=-0.0180, z=0.4883
左手推回多推点 offset : x=-0.0719, y=-0.0180, z=0.4883
左手脱离回拉点 offset : x=-0.0819, y=-0.0180, z=0.4883
右手投放点 offset     : x=-0.1819, y=-0.0680, z=0.6883
左手上方安全高度      : 0.12m
```

## 第一步任务

先做只读锁存脚本，不动手臂：

```text
低头
-> 识别三层 id=3 / 二层 id=2
-> 估计 Tag 在 camera/head/base 下的位置
-> 保存 data/runtime/place_apriltag_target_latest.json
-> 抬头
```

先测两层货架的脖子角度：

```bash
conda activate detect
python apps/place/run_apriltag_lock.py \
  --ws-url ws://192.168.20.102:9091 \
  --neck-test-only \
  --neck-y-values 0.25,0.30,0.35,0.40 \
  --tag-ids 3,2 \
  --tag-size-m 0.020 \
  --transport raw \
  --raw-throttle-ms 500 \
  --settle-seconds 0.8 \
  --show-window
```

三层锁存：

```bash
python apps/place/run_apriltag_lock.py \
  --ws-url ws://192.168.20.102:9091 \
  --shelf-level 3
```

二层锁存：

```bash
python apps/place/run_apriltag_lock.py \
  --ws-url ws://192.168.20.102:9091
```

后续基于该 BASE 位姿测量固定偏移：

```text
Tag -> 左手拉箱接触点
Tag -> 左手拉出方向
Tag -> 左手拉出距离
Tag -> 右手箱口上方点
Tag -> 右手投放点
Tag -> 推回终点
```

## 已验证：左手拉箱接近参数

2026-08-18 实测，二层 `id=2` AprilTag 锁存后，左手示教到“虎口向下扣住箱子前沿”的 TCP pose：

```text
data/poses/place/place_left_pull_grasp_dual.json
```

由该 TCP pose 减去 AprilTag BASE pose 得到固定偏移：

```text
offset_x = -0.0819 m
offset_y = -0.0180 m
offset_z = +0.4883 m
```

左手 orientation 使用 `place_left_pull_grasp_dual.json` 里的 `left.pose.orientation`，不要保持默认手腕姿态：

```text
x =  0.2964303933275394
y = -0.32462723860067083
z = -0.7009908789469625
w =  0.5615674184845025
```

左手手指参数：

```bash
# 扣住/拉箱手型
rosservice call /zj_humanoid/hand/joint_switch/left "{q: [-0.5, 1.2, 0.5, 0.6, 0.6, 0.6]}"

# 左手 home/张开
rosservice call /zj_humanoid/hand/joint_switch/left "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

当前已验证的动作分解：

```text
place_ready_after_grasp_dual
-> 左手到 tag + offset + 上方 12cm
-> 左手切换扣住/拉箱手型
-> 左手下降到 tag + offset，即示教拉出点
```

到拉出点上方 12cm：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.12 \
  --duration 6.0 \
  --max-motion 1.2 \
  --execute-delay 2.0 \
  --execute
```

下降到拉出点：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.00 \
  --duration 3.0 \
  --max-motion 0.2 \
  --execute-delay 2.0 \
  --execute
```

拉出箱子 20cm，现场已验证可用：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.2819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.00 \
  --duration 5.0 \
  --max-motion 0.3 \
  --execute-delay 2.0 \
  --execute
```

## 已验证：右手投放测试参数

右手投放路径要求：

```text
左手正在拉住/拉出箱子，左手 TCP 不能移动。
右手路径 = 当前右手 -> place_right_drop_mid_dual -> 投放点。
右手到投放点后松开塑料袋。
松开后，右手必须原路返回：投放点 -> place_right_drop_mid_dual -> place_ready_after_grasp_dual 的右手 pose。
```

当前右手投放点参数：

```text
offset_x = -0.1819   # 相对箱子前沿抓取点，往身体方向 10cm
offset_y = -0.0680   # 往机器人右侧 5cm
offset_z =  0.6883   # 相对 tag，等价于左手抓取 z + 20cm
```

运行右手到投放点：

```bash
python apps/place/run_right_drop_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --via-file data/poses/place/place_right_drop_mid_dual.json \
  --offset-x -0.1819 \
  --offset-y -0.0680 \
  --z-offset 0.6883 \
  --above-height 0.00 \
  --duration 5.0 \
  --max-motion 1.5 \
  --execute-delay 0 \
  --execute
```

右手松开：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

右手原路返回预放置姿态：

```bash
python apps/place/run_right_drop_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --via-file data/poses/place/place_right_drop_mid_dual.json \
  --target-file data/poses/place/place_ready_after_grasp_dual.json \
  --duration 5.0 \
  --max-motion 1.5 \
  --execute-delay 0 \
  --execute
```

推回箱子并多推 1cm，目标 `offset_x=-0.0719`：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0719 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.00 \
  --duration 5.0 \
  --max-motion 0.3 \
  --execute-delay 2.0 \
  --execute
```

推回后左手恢复路线：

```text
1. 因为推回比原拉出点多推了 1cm，先往身体侧回拉 1cm：offset_x -0.0719 -> -0.0819
2. 从扣住点上抬 12cm：above-height 0.00 -> 0.12
3. 收回到 place_ready_after_grasp_dual 或后续安全点
```

回拉 1cm：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.00 \
  --duration 3.0 \
  --max-motion 0.2 \
  --execute-delay 2.0 \
  --execute
```

上抬 12cm：

```bash
python apps/place/run_left_pull_approach.py \
  --ws-url ws://192.168.20.102:9091 \
  --locked-tag data/runtime/place_apriltag_target_latest.json \
  --offset-x -0.0819 \
  --offset-y -0.0180 \
  --z-offset 0.4883 \
  --above-height 0.12 \
  --duration 4.0 \
  --max-motion 0.3 \
  --execute-delay 2.0 \
  --execute
```

注意事项：

```text
1. `run_left_pull_approach.py` 默认读取 `data/poses/place/place_left_pull_grasp_dual.json` 的左手 orientation。
2. 如果重新贴 AprilTag、换箱子或重摆机器人，需要重新锁存 tag；如果箱子前沿/标签相对位置变化，需要重新示教并重算 offset。
3. 上方 12cm 已验证可用于先对齐手腕姿态，避免下降时推箱。
```

## 风险点

```text
右手持袋是否挡住头部相机看 AprilTag
低头角度是否能同时看到标签和箱口
左手是否有足够自由度拉箱
拉箱过程中箱子是否卡货架
拉出距离是否足够右手放袋
放袋时右手是否碰货架上沿
推回时箱子是否歪斜
MPC 是否因为右手持袋姿态导致躯干扭曲
```

## 备选：箱体轮廓识别

当前仍保留蓝色/白色箱体轮廓识别入口，用于没有 AprilTag 或 Tag 识别失败时辅助判断箱子上沿中点。

运行入口：

```bash
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --show-window
```

头部姿态已合适时：

```bash
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --skip-neck-down \
  --skip-neck-home \
  --show-window
```

调整画面预设点：

```bash
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --center-x-frac 0.50 \
  --center-y-frac 0.50 \
  --show-window
```

备选方案数据输出：

```text
data/place/shelf_box_target_latest.json
data/place/shelf_box_target_latest.png
data/place/shelf_box_debug_latest/
```

核心字段：

```text
target                         放置参考点，相机系 3D
selected_candidate             被选中的目标箱子候选
selected_box_center_distance_px
                               箱子中心到画面预设点距离
evaluated_targets              候选箱子的前沿中点评估结果
rim_corners                    盒口四角
rim_meta                       盒口几何拟合信息
```

备选方案过滤规则：

严格蓝色：

```text
HSV H: 82~110
HSV S: >= 85
HSV V: >= 70
Lab b: <= 112
```

严格纯白：

```text
Lab L: >= 185
HSV S: <= 55
HSV V: >= 170
Lab a: 116~140
Lab b: 116~144
mask 内白色占比: >= 75%
```

现场依据：

```text
真实白箱样本 white_overlap ≈ 0.857
合并背景的大白色 mask white_overlap ≈ 0.543
因此暂定阈值 75%，不用 95%，避免真实白箱因阴影/半透明边缘被误杀。
```

完整箱体过滤：

```text
面积比例合理
宽高比 <= 3.2
高度 >= 画面高度 18%
```

这些过滤只用于排除非目标候选；真正选择哪个箱子只看候选箱子中心到画面预设点的距离。

## 调试判断

候选图：

```text
data/place/shelf_box_debug_latest/candidates.png
```

标签含义：

```text
rank        排序编号
idx         FastSAM 原始候选编号
box_d       候选箱体中心到画面预设点距离
color       blue/white/none
box=1       通过完整箱体过滤
```

最终图：

```text
data/place/shelf_box_debug_latest/annotated.png
```

判断标准：

```text
选中的 mask 是预设点正对的蓝色箱体
红线贴住箱体正对机器人一面的上沿
白点/目标点位于该上沿中点
```

## 风险点

1. 如果货架光照导致目标箱体 `color=none`，需要调整阈值或补光。
2. 如果横向货架边缘被误判为箱体，需要检查 `box=0/1` 和宽高比过滤。
3. 当前输出仍是相机系点，不能直接作为 MPC 目标。
4. 独立放置运动流程尚未完成。
