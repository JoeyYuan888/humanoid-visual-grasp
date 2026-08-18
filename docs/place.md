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
-> 右手到箱口上方
-> 右手下降到投放点
-> 右手松开塑料袋
-> 右手撤回
-> 左手推回箱子
-> 双臂回安全/下垂姿态
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
