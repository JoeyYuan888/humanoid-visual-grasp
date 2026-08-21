# Transport Stage

运输阶段用于识别蓝色盒子的双手抓取点。当前只读识别已经接入；双手运动、夹持、搬运路径还未接入主流程。

## 当前默认识别方案

默认使用项目内置 FoundationPose 箱子 CAD 位姿估计：

```text
FoundationPose RGB-D 6D pose
-> CAD 盒口模型投影
-> 计算左右短边中心抓取点
-> 输出相机系 left/right 抓取点
-> lock_box_grasp_target.py 转换为 BASE 坐标
```

FoundationPose 代码放在：

```text
third_party/foundationpose_crate/
```

原 FastSAM/颜色方案已保留为备份：

```text
apps/transport/legacy/run_box_grasp_point_fastsam.py
apps/transport/run_box_grasp_point.py
```

## 旧版备份识别方案

```text
FastSAM 分割盒子候选
-> 选择蓝色盒子 mask
-> depth-rim + side-mid 几何拟合盒口
-> 红线 = 盒子前沿
-> 绿线 = 后沿鲁棒拟合，忽略盒内塑料袋局部凸起
-> 左右抓取点 = 左右侧边中点
-> 如果抓取点落到 mask 外，沿侧边回退到最近 mask 内有效点
```

该脚本不发手臂运动命令。

## 现场命令

机器人需要先启动 rosbridge：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

默认 FoundationPose 低头、识别、退出后抬头：

```bash
conda activate foundationpose_detect
python apps/transport/run_foundationpose_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

Transport 主流程默认使用 FoundationPose，并继续执行到箱子两侧外扩 5cm 的靠近点：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

主流程动作顺序：

```text
低头识别箱子左右抓取点
-> 最近 5 帧 valid left/right 抓取点分别取平均
-> 相机系抓取点锁存到 BASE
-> 设置 MPC running mode
-> home / 当前姿态到 transport_pregrasp_dual
-> 生成往身体 17cm、左右各向外 10cm、z=抓取点+0.38m 的外扩等待点
-> 设置 MPC running mode
-> transport_pregrasp_dual 到外扩等待点
-> 开启导纳
-> 用 /wa/points_seq_tracking_with_admittance 单段夹紧到 clamp_offset
-> 关闭导纳
```

默认靠箱参数：

```text
outside_offset = 0.10 m
clamp_offset   = 0.03 m
clamp_offsets  = ""      # 默认不用分段；仅调试时手动传入
clamp_control  = admittance
body_offset    = -0.17 m   # BASE x 负方向，往身体 17cm；相比上一版向身体反方向外移 2cm
side_z_offset  = 0.38 m    # 当前夹紧点上移 38cm；相比上一版下调 2cm
motion_duration = 5.0 s
side_motion_duration = 10.0 s   # 预抓取到两侧靠近点降速 50%
carry_lift = 0.30 m             # 夹紧/手指补夹后抬高 30cm
carry_waist_joints = [-0.3, 0.3, 0.0, 0.0]  # 回收后设置 3-6 四个身体关节
carry_pullback = -0.20 m        # 抬高后 BASE x 负方向回收 20cm
```

当前箱子搬运手型：

```text
left_hand_q  = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
right_hand_q = [0.2, 0.9, 0.35, 0.45, 0.56, 0.65]
```

当前主流程默认已自动执行手指补夹、抬高 30cm、往身体回收 20cm、调整腰部搬运姿态。调试时可用 `--skip-hand-adjust`、`--skip-carry-lift`、`--skip-carry-pullback` 跳过。

搬运到目标货架/放置区域后，放箱并收回双臂：

```bash
python apps/transport/run_transport_place_return.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

该流程顺序：

```text
当前搬运姿态 -> data/poses/transport/transport_place_dual.json
-> 双手复位/松开
-> 双手水平外扩 10cm，先离开箱子侧壁
-> 双手上抬 12cm，避开箱子/桌面
-> data/poses/transport/transport_pregrasp_dual.json
-> data/poses/transport/transport_home_dual.json
```

注意：`transport_place_dual.json` 是实际放置/夹紧点，已经去掉左右外扩偏移；`transport_box_side_approach_latest.json` 是搬运抓取阶段的外扩点，放置 return 默认不再经过它，避免松手后往低处运动撞桌面。只有调试需要时才加 `--use-outside-retreat`。

## 当前搬运总流程

完整搬运链路从搬运开始点导航开始：

```bash
python apps/transport/run_transport_delivery_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window \
  --execute
```

该入口把四段串起来：

```text
1. run_navigation_flow.py --goal transport_start_area
   导航到底盘地图上的搬运开始点

2. run_transport_flow.py
   当前位置识别箱子、双手夹紧、补夹、抬高、回收

3. run_navigation_flow.py --goal transport_place_area
   导航到底盘地图上的搬运结束点

4. run_transport_place_return.py
   放下箱子、松手后先外扩再上抬、经 transport_pregrasp_dual 回 transport_home_dual
```

如果机器人已经由人工移动到搬运起点，跳过起点导航：

```bash
python apps/transport/run_transport_delivery_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window \
  --skip-start-navigation \
  --execute
```

只测试终点导航和放置/return：

```bash
python apps/transport/run_transport_delivery_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-transport \
  --skip-start-navigation \
  --execute
```

只测试搬运抓取，不导航、不放置：

```bash
python apps/transport/run_transport_delivery_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window \
  --skip-navigation \
  --skip-return \
  --execute
```

如果导纳临时不可用，回退普通 points：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --clamp-control points \
  --execute
```

如果机器人已经在外扩等待点，只想验证导纳夹紧 1cm，可临时传 `--clamp-offsets`：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --clamp-only \
  --clamp-control admittance \
  --clamp-offsets 0.09 \
  --execute
```

当前现场进度：导纳夹紧到 3cm 已验证，双手同步抬高 30cm、往身体回收 20cm、腰部姿态调整已并入主流程；搬运直腰姿态仍在优化。完整记录见 `docs/transport.md`。

如果只想停在预抓取点：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-side-approach \
  --execute
```

旧 FastSAM 备份方案：

```bash
conda activate detect
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --show-window
```

如果头部已经在观察姿态，不让脚本移动脖子：

```bash
python apps/transport/run_foundationpose_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-neck-down \
  --skip-neck-home \
  --show-window
```

## 输出

```text
data/transport/box_grasp_target_latest.json
data/transport/box_grasp_target_latest.png
data/transport/box_grasp_debug_latest/
data/transport/foundationpose_box_grasp_debug_latest/
data/runtime/transport_box_grasp_camera_latest.json
data/runtime/transport_box_grasp_target_latest.json
data/runtime/transport_box_side_approach_latest.json
```

关键 JSON 字段：

```text
objects[side=left/right]       左右抓取点，相机系 3D
source                         foundationpose / fastsam / color_depth_rim
rim                            盒口四边形
rim_corners                    top_left/top_right/front_left/front_right
rim_source                     期望为 side_mid
rim_meta.front_line            期望包含 hough_front_robust_top
rim_meta.handle_point_adjustment
                               如果抓取点被拉回 mask 内，会记录 from/to
```

## 调试图

```text
sam_candidates.png             FastSAM 候选
sam_selected_mask.png          选中的盒子 mask
fastsam_rim.png                最终盒口线和左右抓取点
rim_edges.png                  RGB/深度边缘
rgb.png / depth_vis.png        原始图和深度可视化
```

判断标准：

```text
红线贴住盒子前沿
绿线贴住后沿主边，不被盒内塑料袋局部凸起拉偏
左右抓取点在两侧边中部，并且在 mask/盒体边界内
```

FoundationPose 判断标准：

```text
latest.png 里 CAD 红/绿盒口投影贴住真实箱子边缘
left/right 抓取点分别落在左右短边中心
latest_grasp_points.json 中 valid=true
robot_execution_allowed=true 表示两侧抓取点没有被遮挡
当前接入层默认使用最近 5 帧 valid 抓取点平均值作为最终相机系抓取点
当前接入层只要 left/right 都 valid，就会把两侧 z 统一为两侧 z 平均值
```

## 深度快照

如果识别不稳定，先采集低头后的 RGB/深度图，不做盒子检测：

```bash
python apps/transport/capture_depth_snapshot.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

输出：

```text
data/transport/depth_snapshot_latest/rgb.png
data/transport/depth_snapshot_latest/depth_vis.png
data/transport/depth_snapshot_latest/depth_edges.png
data/transport/depth_snapshot_latest/near_mask.png
data/transport/depth_snapshot_latest/overlay.png
data/transport/depth_snapshot_latest/depth_raw.npy
```

## 搬运动作准备

动作阶段先记录双手安全点，不直接抓盒子。

记录当前双手 home/safe 点：

```bash
python tools/capture/capture_dual_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --name transport_home \
  --include-joints \
  --output data/poses/transport/transport_home_dual.json
```

进入示教后，把双手摆到盒子两侧上方的预抓取姿态，再记录：

```bash
python tools/capture/capture_dual_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --name transport_pregrasp \
  --include-joints \
  --output data/poses/transport/transport_pregrasp_dual.json
```

如果左右手位置不够对称，先只做离线小幅微调建议：

```bash
python tools/capture/tune_dual_pose_symmetry.py \
  data/poses/transport/transport_pregrasp_dual.json \
  --max-position-delta 0.02 \
  --name transport_pregrasp_tuned \
  --output data/poses/transport/transport_pregrasp_tuned_dual.json
```

该脚本只调整左右末端 position 的对称性：

```text
x/z 取左右平均
y 取左右绝对值平均，左正右负
每个轴最大只改 --max-position-delta
orientation 保持不动
不会连接 ROS，不会发运动
```

注意：微调文件里的 `mpc_state` 仍来自原始示教姿态。要想用身体/手臂 joint 约束一比一复现，必须先小步移动到微调后的 pose，再重新采集一次：

```bash
python tools/capture/capture_dual_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --name transport_pregrasp_tuned_recaptured \
  --include-joints \
  --output data/poses/transport/transport_pregrasp_tuned_recaptured_dual.json
```

把双手摆到实际夹持姿态，再记录：

```bash
python tools/capture/capture_dual_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --name transport_grasp \
  --include-joints \
  --output data/poses/transport/transport_grasp_dual.json
```

记录顺序：

```text
home/safe
-> pregrasp
-> grasp
-> 双手开合参数
-> 再做 MPC dry-run
```

## 注意事项

1. 运输阶段是双手盒子搬运，不复用塑料袋单手抓取参数。
2. 当前输出是相机系坐标，不能直接发 MPC；后续需要接 CAM2HEAD + live TF 到 BASE。
3. 双手运动必须同步规划，否则会推盒子。
4. 当前方案假设目标是蓝色盒子；非蓝色箱体需要换候选过滤策略。
