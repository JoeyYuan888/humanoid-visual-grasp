# Place Stage

放置阶段用于把右手抓取的塑料袋放入货架箱子。当前导航步骤先不接入：机器人抓起塑料袋后，由人工移动到货架放置点，再运行放置阶段调试流程。

## 当前目标流程

```text
右手持塑料袋，人工移动机器人到货架放置点
-> MPC running mode
-> 低头
-> 头部相机识别箱子把手处 AprilTag
-> AprilTag 相机系位姿 -> HEAD -> BASE
-> 根据 Tag/箱子几何偏移生成左手拉箱点、右手投放点
-> 抬头
-> 左手到拉箱预抓取点
-> 左手接近 AprilTag/把手区域
-> 左手夹/勾住箱子前沿或把手
-> 左手向身体侧拉出箱子
-> 右手移动到箱口上方
-> 右手下降到放置点
-> 右手松开塑料袋
-> 右手撤回
-> 左手把箱子推回货架
-> 左手撤回，双臂回安全/下垂姿态
```

## AprilTag 定位参数

当前贴在货架箱子把手处的辅助定位标签：

```text
Tag family : AprilTag 36h11
Tag size   : 20 mm = 0.020 m
三层 Tag id: 3, neck_y=0.25
二层 Tag id: 2, neck_y=0.40
```

要求：

```text
标签必须贴在目标箱子正面把手附近，平面尽量和箱子正面一致。
三层用 id=3，二层用 id=2；同一层画面内不要出现重复 id。
调试图底部会标注 `shelf / neck_y / target_ids`，避免混用 0.25 和 0.40 的旧图。
相机识别时优先低头观察，确认右手塑料袋不会挡住标签。
```

## 第一阶段：只做 AprilTag 锁存

先不动手臂，只验证视觉链路：

```text
低头
-> 识别三层 id=3 / 二层 id=2
-> 估计 Tag 在 camera/head/base 下的位置
-> 保存 BASE 锁存结果
-> 抬头
```

### 1.1 先测两层货架的脖子角度

货架有两层，每层都必须能看见对应 AprilTag。先只测试不同低头角度，不做 BASE 锁存：

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

输出：

```text
data/runtime/place_apriltag_neck_test/summary.json
data/runtime/place_apriltag_neck_test/neck_y_*_raw.png
data/runtime/place_apriltag_neck_test/neck_y_*_annotated.png
data/runtime/place_apriltag_neck_test/neck_y_*.json
```

脚本会等 neck 到位并额外等待 `--settle-seconds` 后，再读取下一帧。这样可以避免使用运动过程中的旧帧。

判断标准：

```text
每个目标货架层至少有一个 neck_y 能稳定检测到对应 id：三层 id=3，二层 id=2。
调试图中 Tag 边框完整、无遮挡、不是画面边缘。
优先选能同时看到 Tag 和箱口的位置。
当前定为：三层 `neck_y=0.25`，二层 `neck_y=0.40`。
```

### 1.2 确认角度后做 BASE 锁存

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

层级默认参数已经写入脚本：

```text
tag-size-m  = 0.020
transport   = raw
raw throttle= 500ms
settle      = 0.8s

相机连接顺序固定为：低头完成 -> 连接相机 -> 获取 raw 图 -> 识别/保存 -> 抬头。

二层: tag-id=2, neck-down-y=0.40
三层: tag-id=3, neck-down-y=0.25
```

计划输出：

```text
data/runtime/place_apriltag_target_latest.json
data/runtime/place_apriltag_debug_latest.png
```

建议 JSON 字段：

```text
tag.family                  tag36h11
tag.id                      4
tag.size_m                  0.020
camera.tag_pose             AprilTag 在相机坐标系下的 pose
head.tag_pose               AprilTag 在 HEAD 坐标系下的 pose
base.tag_pose               AprilTag 在 BASE 坐标系下的 pose
base.pull_pregrasp_pose     左手拉箱预抓取参考点，后续测量后写入
base.pull_contact_pose      左手接触箱子/把手参考点，后续测量后写入
base.pull_out_pose          左手拉出终点，后续测量后写入
base.drop_pose              右手投放参考点，后续测量后写入
```

## 第二阶段：测量固定偏移

AprilTag 只能给出箱子正面局部位姿，真实动作还需要现场测量固定偏移。先用小步 dry-run 和试教记录建立这些偏移：

```text
Tag -> 左手拉箱接触点
Tag -> 左手拉出方向
Tag -> 左手拉出距离
Tag -> 右手箱口上方点
Tag -> 右手投放点
Tag -> 推回终点
```

建议初始测量顺序：

```text
1. 右手保持 QR 展示/持袋姿态，低头看标签，确认是否遮挡。
2. 只动左手，记录能安全碰到箱子前沿/把手的预抓取点。
3. 左手小步靠近箱子，确认接触点。
4. 左手沿身体方向每次拉 2~3 cm，测量最小可投放拉出距离。
5. 右手从持袋姿态移动到箱口上方，确认不会撞货架。
6. 右手下降到投放高度，松手。
7. 左手反向推回箱子。
```

## 第三阶段：接入手臂运动

动作拆成多段，先不追求一条指令完成：

```text
锁存 Tag BASE
-> 左手到 pull_pregrasp
-> 左手到 pull_contact
-> 左手 pull_out
-> 右手到 drop_above
-> 右手到 drop_pose
-> 右手松手
-> 右手撤回
-> 左手 push_back
-> 双臂回安全点
```

每段都需要：

```text
先 dry-run
检查路径累计长度、max_z、左右手保持侧 pose
确认急停
再 execute
```

## 需要验证的风险点

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

## 现场固定参数待测

```text
neck_down_y                  待确认，先沿用 0.35 或现场安全值
pull_out_distance_m          待测
pull_contact_offset_from_tag 待测
drop_offset_from_tag         待测
drop_above_height_m          待测
right_hand_open_q            沿用抓取流程的放开参数
left_hand_pull_q             待测，可能只需夹/勾住，不一定完整闭合
```

## 旧方案：箱体轮廓识别

以下方案用于没有 AprilTag 时识别货架上的目标蓝色/白色箱子，并输出该箱子正对机器人一面的上沿中点。当前正式放置路线优先使用 AprilTag；轮廓识别保留为调试/备选。

### 识别逻辑

```text
机器人预设点位正对某个货架格/箱子
-> 画面中的预设点通常为中心点
-> FastSAM 生成候选 mask
-> 使用严格颜色过滤，只保留蓝色或纯白箱体 mask
-> 使用尺寸/形状过滤去掉横条、局部碎片
-> 在剩余蓝色箱体候选中，只按候选箱子中心到预设点的距离选择目标箱子
-> 对选中箱子运行 depth-rim + side-mid
-> 输出 front_left/front_right 的中点，作为放置目标参考点
```

注意：画面预设点只用于决定选哪个箱子；最终输出点是被选中箱子的前沿上沿中点。

### 现场命令

机器人需要先启动 rosbridge：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

低头、识别、显示窗口、退出后抬头：

```bash
conda activate detect
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --show-window
```

如果头部已经在观察姿态，不让脚本移动脖子：

```bash
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --skip-neck-down \
  --skip-neck-home \
  --show-window
```

如果机器人预设点位正对的箱子不在画面中心，用比例调整预设点：

```bash
python apps/place/run_shelf_box_target.py \
  --ws-url ws://192.168.20.102:9091 \
  --rim-fit-mode side-mid \
  --center-x-frac 0.50 \
  --center-y-frac 0.50 \
  --show-window
```

### 输出

```text
data/place/shelf_box_target_latest.json
data/place/shelf_box_target_latest.png
data/place/shelf_box_debug_latest/
```

关键 JSON 字段：

```text
target                         前沿上沿中点，相机系 3D
selected_candidate             被选中的目标箱体候选
selected_box_center_distance_px
                               选中箱子中心到画面预设点的像素距离
evaluated_targets              已评估候选的前沿中点和距离信息
rim_corners                    top_left/top_right/front_left/front_right
rim_meta                       几何拟合调试信息
```

### 调试图

```text
candidates.png                 候选框，显示 rank/idx/box_d/blue/box
selected_mask.png              最终选中的箱子 mask
annotated.png                  最终前沿中点和盒口线
rgb.png / depth_vis.png        原始图和深度可视化
```

候选图标签含义：

```text
rank                           当前排序编号
idx                            FastSAM 原始候选编号
box_d                          候选箱体中心到画面预设点距离
color=blue/white/none          通过的颜色类别
box=1                          通过完整箱体形状过滤
```

### 当前过滤规则

放置阶段使用比运输阶段更严格的颜色判断，避免把灰色货架边/横条误识别成箱子。

蓝色：

```text
HSV H: 82~110
HSV S: >= 85
HSV V: >= 70
Lab b: <= 112
```

纯白：

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

完整箱体候选还需要满足：

```text
面积比例合理
宽高比 <= 3.2
高度 >= 画面高度 18%
```

### 注意事项

1. 放置阶段只用画面预设点选择目标箱子，不按颜色深浅或面积大小决定放哪个箱子。
2. 如果目标箱子因为光照导致 `color=none`，需要调整严格颜色阈值或现场补光。
3. 如果出现横向长条被选中，优先检查 `box=0/1` 和宽高比过滤。
4. 当前输出是相机系坐标，接放置运动前还需要 CAM2HEAD + live TF 转 BASE。
5. 该轮廓方案不参与当前 AprilTag 正式放置主线，除非 Tag 识别失败需要备选。
