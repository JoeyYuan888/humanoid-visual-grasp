# Grasp Stage

本文档只描述“塑料袋抓取”本身，不描述抓后 OCR/QR、导航、货架放置、箱子拉出/推回。

当前抓取阶段边界：

```text
input : 机器人在抓取起始姿态，桌面/地面上有塑料袋，头部相机可见目标
logic : 低头锁存目标 -> MPC 安全路径 -> 右手到抓取点 -> 手掌闭合 -> 压力确认
output: 右手抓住塑料袋，机器人停在抓取点或交给后续抓后识别流程
```

抓取之后的内容分别看：

```text
docs/post_grasp_identification.md   抓后 OCR/QR 识别
docs/place.md                       货架放置
docs/flowchart.md                   总流程图
docs/troubleshooting.md             故障处理
```

## 固定入口

只执行抓取阶段，手掌闭合后停在抓取点：

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode none \
  --max-z 1.70 \
  --execute
```

如果只想验证抓取锁存和抓取动作，不需要重新检测目标，可复用上一次锁存：

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --skip-lock \
  --return-mode none \
  --max-z 1.70 \
  --execute
```

抓取后需要继续 OCR/QR 时，看 `docs/post_grasp_identification.md`；需要货架放置时，看 `docs/place.md`。

## 机器人端准备

在 `huimin1.4` 容器内启动 rosbridge：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

电脑端固定连接：

```text
ws://192.168.20.102:9091
```

确认 MPC running mode：

```bash
rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: true"
```

## 运行环境

电脑端使用 conda 环境：

```bash
conda activate detect
```

抓取视觉默认使用 CUDA：

```bash
nvidia-smi
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO CUDA')"
```

要求：

```text
torch.cuda.is_available() == True
```

安装顺序：

```bash
pip install -r requirements-torch-cu128.txt
pip install --no-deps -r requirements.txt
```

不要省略 `--no-deps`，避免依赖安装覆盖 CUDA 版 PyTorch 或把 `opencv-contrib-python` 换成普通 `opencv-python`。

## Head Neck

抓取阶段低头角度：

```text
neck_joint = [0.0, 0.35]
```

抬头：

```text
neck_joint = [0.0, 0.0]
```

说明：实测 `0.39/0.40` 容易接近限位或造成抬头卡顿；当前抓取锁存统一使用 `0.35`。

## Plastic Bag Detection

模型：

```text
models/yolo/best.pt
```

核心参数：

```text
YOLO_CONF = 0.25
OBJECT_MIN_CONF = 0.30
YOLO_IMGSZ = 512
REQUIRE_CUDA = True
YOLO_HALF = True
```

抓取点像素：

```text
bbox x = 0.50
bbox y = 0.333
```

含义：bbox 上方 2/3 区域的中点。

锁存目标选择策略：

```text
多帧连续性过滤 FP
默认选择 bbox 中心最靠近画面下半部中点的稳定目标
--min-lock-hits 3
--lock-match-distance 0.12
--lock-target-policy image_center
```

白色塑料袋过曝时使用轻量高光抑制：

```bash
--highlight-suppression mild
```

如果现场光照正常或漏检，回退：

```bash
--highlight-suppression none
```

## CAM2HEAD

当前默认手眼矩阵：

```text
data/calibration/cam2head_vendor_new_20260803.json
```

矩阵：

```text
[[-0.02589883, -0.28658041,  0.95770607,  0.07804458],
 [-0.99962985,  0.01540832, -0.02242183,  0.03919847],
 [-0.00833098, -0.95793228, -0.28687339,  0.12356851],
 [ 0.0,         0.0,         0.0,         1.0]]
```

锁存输出：

```text
data/runtime/mpc_locked_target_latest.json
```

## TCP Offset

当前默认抓取 profile：

```text
data/poses/grasp_profile_tuned_with_orientation.json
```

固定参数：

```text
offset_x =  0.045
offset_y = -0.095
offset_z =  0.35
grasp_height = 0.02
orientation_file = data/poses/mpc_grasp_tuned_pose_right.json
orientation_apply = prealign
```

目标计算：

```text
target_tcp.x = object_base.x + offset_x
target_tcp.y = object_base.y + offset_y
target_tcp.z = object_base.z + grasp_height + offset_z
```

当前最终 TCP 高度：

```text
target_tcp.z = object_base.z + 0.37
```

保留回退 profile：

```text
data/poses/grasp_profile_legacy_no_orientation.json
```

只有明确要回退无姿态版本时使用：

```bash
--grasp-profile legacy_no_orientation
```

## Right Hand

右手张开：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

右手闭合抓塑料袋：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [0.5, 0.8, 0.84, 0.84, 0.35, 0.35]}"
```

压力话题：

```text
/zj_humanoid/hand/finger_pressures/right
```

当前压力阈值采用“高于空手噪声即可继续”的策略：

```text
grasp_pressure_threshold = 0.00
```

不做补夹，不修改手掌参数。

## 关键 Pose 文件

必须存在：

```text
data/poses/mpc_via1_pose_right.json
data/poses/mpc_via2_pose_right.json
data/poses/mpc_via3_pose_right.json
data/poses/mpc_grasp_tuned_pose_right.json
```

说明：

```text
via1/via2/via3 是到抓取区域的安全路径。
mpc_grasp_tuned_pose_right.json 只提供抓取姿态 orientation，不作为位置目标。
```

## 抓取阶段流程

```text
1. 设置 MPC running mode。
2. MPC neck 低头。
3. RealSense 采集图像。
4. YOLO 检测塑料袋。
5. 多帧连续性过滤，选择画面下半部中点附近的稳定目标。
6. 取 bbox 上方 2/3 中点深度。
7. camera -> HEAD -> BASE，保存锁存目标。
8. MPC neck 抬头。
9. 右手张开。
10. via1 -> via2 -> via3。
11. via3 -> 视觉抓取点，使用 tuned orientation。
12. 右手闭合。
13. 压力检查。
```

抓取阶段完成状态：

```text
右手持袋
机器人在抓取点
后续交给抓后识别或人工调试流程
```

## 分段调试命令

只看头部相机/YOLO：

```bash
python apps/grasp/test_yolo.py
```

只低头锁存：

```bash
python apps/grasp/run_perception_lock.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

只走 via1->via2->via3：

```bash
python apps/grasp/run_visual_grasp_test.py \
  --ws-url ws://192.168.20.102:9091 \
  --use-locked-target data/runtime/mpc_locked_target_latest.json \
  --via-file data/poses/mpc_via1_pose_right.json \
  --via-file data/poses/mpc_via2_pose_right.json \
  --via-file data/poses/mpc_via3_pose_right.json \
  --stop-at-last-via \
  --use-joints \
  --max-motion 2.0 \
  --max-z 1.70 \
  --duration 5.0 \
  --execute-delay 0 \
  --execute \
  --confirm-target
```

只从 via3 到抓取点：

```bash
python apps/grasp/run_visual_grasp_test.py \
  --ws-url ws://192.168.20.102:9091 \
  --use-locked-target data/runtime/mpc_locked_target_latest.json \
  --include-descend \
  --no-auto-lift \
  --max-motion 1.2 \
  --max-z 1.70 \
  --duration 5.0 \
  --execute-delay 0 \
  --execute \
  --confirm-target
```

记录右手单臂 pose：

```bash
python tools/capture/capture_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --arm right \
  --include-joints \
  --output data/poses/<name>.json
```

## 抓取失败处理

压力检查失败时：

```text
1. 不补夹。
2. 不改手掌 q。
3. 回 via1。
4. 打开手掌。
5. 重新低头锁存。
6. 最多重试一次。
```

## 禁止事项

```text
1. 不要恢复 SDK 手臂路线。
2. 不要在 grasp.md 维护 OCR/QR、导航、货架放置或箱子拉出流程。
3. 不要随意修改已固定 offset/profile，除非现场重新验证。
4. 不要用旧 CSV 配新的头部姿态重算目标。
5. 不要提交 data/samples/grasp_data_*.csv 运行样本，除非明确作为离线样本。
6. 不要提交大模型权重到 GitHub 普通仓库。
```
