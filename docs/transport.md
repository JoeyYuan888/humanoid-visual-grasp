# Box Transport Stage

运输阶段负责双手搬运箱子。当前默认使用 FoundationPose + CAD 模型识别箱子 6D pose，并输出左右抓取点；原 FastSAM/颜色方案保留为 legacy 回退。

## 阶段边界

```text
input : 头部相机 RGB/depth、camera_info、箱子 CAD 模型
logic : FoundationPose 6D pose、CAD 盒口投影、左右短边中心抓取点
output: 相机系左右抓取点，再转换为 BASE
```

不和塑料袋单手抓取流程混写：

```text
apps/grasp/      塑料袋抓取、OCR/QR、放回
apps/transport/ 蓝色盒子识别、双手搬运
```

## 当前默认识别方案

```text
FoundationPose RGB-D 位姿估计
-> CAD 盒口模型投影
-> 左右短边中心作为双手抓取点
-> 输出 camera frame JSON
-> lock_box_grasp_target.py 使用 CAM2HEAD + TF 转 BASE
```

代码位置：

```text
apps/transport/run_foundationpose_box_grasp_point.py
third_party/foundationpose_crate/
```

旧方案备份：

```text
FastSAM 分割候选盒体
-> 蓝色候选筛选
-> depth-rim + side-mid 几何
-> 红线拟合盒子前沿
-> 绿线用后沿多数边界鲁棒拟合，忽略盒内物体局部凸起
-> 左右侧边中点作为双手抓取点
-> 抓取点如落到 mask 外，自动回退到最近 mask 内有效侧边点
```

## 运行入口

默认 FoundationPose：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

默认主流程会继续到箱子两侧靠近点：

```text
低头识别
-> 相机系左右抓取点转 BASE
-> 设置 MPC running mode
-> 到 transport_pregrasp_dual
-> 按往身体 25cm、左右各向外 10cm、z=抓取点+0.05m 生成外扩等待点
-> 设置 MPC running mode
-> 到外扩等待点
-> 分段夹紧: 8cm -> 4cm -> 3cm -> 2cm
-> 每段后检测左右手指尖压力，左右都达标则停止继续收紧
```

默认靠箱参数：

```text
outside_offset = 0.10 m
clamp_offset   = 0.02 m
clamp_offsets  = 0.08, 0.04, 0.03, 0.02 m
clamp_pressure_abs_threshold = left/right >= 0.15
body_offset    = -0.25 m   # BASE x 负方向，往身体 25cm
side_z_offset  = 0.05 m    # 相比原塑料袋 +0.35m 再下 30cm
motion_duration = 5.0 s
side_motion_duration = 10.0 s   # 预抓取到两侧靠近点降速 50%
```

只停在预抓取点：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-side-approach \
  --execute
```

只跑 FoundationPose 识别：

```bash
python apps/transport/run_foundationpose_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

旧 FastSAM 备份方案：

```bash
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --show-window
```

当前头部姿态已合适时：

```bash
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --skip-neck-down \
  --skip-neck-home \
  --show-window
```

## 数据输出

```text
data/transport/box_grasp_target_latest.json
data/transport/box_grasp_target_latest.png
data/transport/box_grasp_debug_latest/
data/transport/foundationpose_box_grasp_debug_latest/
data/runtime/transport_box_grasp_camera_latest.json
data/runtime/transport_box_grasp_target_latest.json
data/runtime/transport_box_side_approach_latest.json
```

核心字段：

```text
objects                         left/right 抓取点，相机系
source                          foundationpose / legacy backend
rim_corners                     盒口四角
rim_meta.front_line             前沿/后沿拟合信息
rim_meta.handle_point_adjustment
                                抓取点回退到 mask 内时记录 from/to
```

## 后续接入

```text
1. 使用 FoundationPose 识别左右抓取点。
2. 将相机系左右抓取点转换到 BASE。
3. 复现双手预抓取姿态。
4. 从预抓取点移动到箱子两侧外扩靠近点。
5. 后续接入双手闭合、压力确认和搬运过程约束路径。
```

## 风险点

1. FoundationPose 依赖 CAD 尺寸、相机内参和第一帧 mask；箱子型号变化时要重新建模或换配置。
2. 双手路径不能分开执行，否则容易推盒子。
3. 手掌压力阈值不能复用塑料袋抓取参数。
4. 相机系点不能直接发送 MPC，必须先通过 CAM2HEAD + TF 锁存到 BASE。
5. 旧 FastSAM/颜色方案保留用于现场回退，不作为默认主流程。
