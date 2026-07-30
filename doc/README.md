# 机器人头部 RealSense 抓取视觉项目

本项目用于机器人静态抓取前的视觉感知测试：通过头部 RealSense 获取 RGB、aligned depth，用 YOLO 检测塑料袋位置，并计算每个目标在相机坐标系下的 3D 坐标。

当前业务方案已经调整为：

```text
桌面阶段：识别物体位置和 3D 坐标
抓取阶段：根据 3D 坐标抓取物体
检查阶段：把已抓物体移动到头部相机前
识别阶段：近距离识别二维码
绑定阶段：把二维码文本绑定到已抓取物体
```

所以：**抓取前不要求二维码已经识别成功**。抓前最重要的是 `valid=True` 和稳定的 `x_mm/y_mm/z_mm`。

## 当前保留文件

核心代码：

```text
run_grasp.py                    # 整套视觉程序入口
robot_grasp/main.py             # ROS/显示/日志编排
robot_grasp/vision_pipeline.py  # 核心视觉管线：YOLO + depth + QR 记忆
robot_grasp/ros_client.py       # rosbridge RGB/raw/depth 订阅
robot_grasp/detector.py         # YOLO 检测封装
robot_grasp/depth_utils.py      # 深度图取值和 2D->3D
robot_grasp/qr_detector.py      # QR 解码器
robot_grasp/qr_worker.py        # 后台 QR 解码线程
robot_grasp/grasp_flow.py       # 抓取目标选择、抓后 QR 绑定逻辑
robot_grasp/hand_utils.py       # 手指关节限位和安全开合姿态
robot_grasp/logger.py           # CSV 记录
```

调试脚本：

```text
run_grasp.py               # 主程序入口
robot_grasp/               # 主程序核心模块
tools/                     # 测试、调试、干跑、性能分析脚本
handeye_calibration/       # 手眼标定工具、文档、点对数据和标定结果
doc/                      # 项目文档、教程、路线记录和旧文档备份
paths/                     # 示教路径/脖子姿态配置
data/                      # 运行输出 CSV/截图等数据
```

当前文档：

```text
doc/README.md
doc/视觉抓取部署教程-从感知到MPC运动.md    # 教程索引
doc/视觉抓取部署教程-SDK路线.md
doc/视觉抓取部署教程-MPC路线.md
doc/SDK_ROUTE_SNAPSHOT_20260721.md
doc/MPC_ROUTE_ISSUES_20260721.md
handeye_calibration/HAND_EYE_ALIGNMENT_PLAN.md
handeye_calibration/test_cam2head_candidate.py
doc/mpc_interface_status.json
doc/人形机器人二次开发接口讲解与实操课程手册.md
doc/WA型号-MPC使用接口文档-外部 副本.md
```

旧文档已移动到：

```text
doc/backup_old_docs_20260720/
```

已保存路径/姿态：

```text
paths/neck_look_down.json       # 头部 RealSense 低头看桌面视觉位
paths/neck_home.json            # 头部官方 go_home 复位，手臂安全回收后使用
paths/teach_path_right_arm.json # 右臂 P1->P2->P3 安全预抓取路径
paths/teach_path_right_arm_return.json # 右臂 P3->P2->P1 安全回收路径
```

## 快速开始

本项目后续统一使用 `detect` conda 环境：

```bash
conda activate detect
```

如果环境重装过，按下面命令恢复依赖。注意：必须安装 CUDA 版 PyTorch，不能安装 CPU 版。

```bash
/home/hmit/miniconda3/envs/detect/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
/home/hmit/miniconda3/envs/detect/bin/python -m pip install ultralytics roslibpy zxing-cpp
```

检查：

```bash
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
python -c "import ultralytics, roslibpy, zxingcpp; print('deps ok')"
```

`torch.__version__` 应带 `+cu128`。RTX 5060 Laptop GPU 是 `sm_120`，旧的 `+cu121` wheel 只支持到 `sm_90`，会报 `CUDA error: no kernel image is available for execution on the device`。如果 `torch.cuda.is_available()` 是 `False`，说明当前终端/容器看不到 GPU，本项目会拒绝退回 CPU。

在项目根目录运行：

```bash
python run_grasp.py
```

窗口按键：

- `q`：退出
- `s`：保存当前采样到 `data/grasp_data_*.csv`
- 鼠标左键：点击测距，并记录点击点 3D 坐标

保存数据后分析：

```bash
python tools/analyze_perf.py
```

如果要指定某个 CSV：

```bash
python tools/analyze_perf.py data/grasp_data_xxx.csv
```

## 推荐调试顺序

遇到卡顿、没深度、没二维码时，不要一上来跑整套。按下面顺序拆开看：

### 1. 只看 ROS 图像/深度流

```bash
python tools/debug_ros_streams.py
```

它不会加载 YOLO/QR，只打印：

```text
rgb=...fps age=...ms | raw=...fps age=...ms | depth_msg=...fps depth=...fps age=...ms
```

判断：

- `rgb` 低：主 RGB 或 rosbridge/network 有问题
- `depth_msg` 低：机器人端深度发布/网络/rosbridge 有问题
- `depth_msg` 正常但 `depth` 低：本地 compressedDepth 解码压力大
- `age` 很大：当前用的是旧帧，会影响坐标

### 2. 只看 YOLO

```bash
python tools/test_yolo.py
```

用于确认模型、GPU、置信度、检测帧率。这个脚本已经支持降低检测频率，显示仍用最新画面。

### 3. 只看二维码

```bash
python tools/test_qr.py
```

用于确认 QR backend、raw RGB 画质、ROI 是否能扫到码。

注意：当前最终业务是抓后近距离扫码，所以桌面阶段 QR 偶尔漏检不一定影响抓取。桌面扫码仍可作为调试参考。

### 4. 单独跑完整视觉管线

```bash
python tools/debug_vision_pipeline.py
```

不开窗口，只打印结果：

```bash
python tools/debug_vision_pipeline.py --no-window
```

这个脚本和 `run_grasp.py` 共用 `robot_grasp/vision_pipeline.py`，所以它是当前最推荐的视觉专项 debug 入口。

### 5. 从 CSV 回放目标选择

```bash
python tools/debug_select_target.py
```

如果后续模型类别名明确，也可以只选某一类：

```bash
python tools/debug_select_target.py --label plastic_bag
```

它会读取最新 `data/grasp_data_*.csv` 的 `object_results`，调用 `select_grasp_target()`，验证业务层会选哪个物体去抓。

### 6. 只检查 SDK/手掌接口

```bash
python tools/debug_sdk_interfaces.py
```

这个脚本只查服务和话题是否在线，不调用运动服务，不会让机器人动作。

### 7. SDK 抓取干跑

```bash
python tools/run_sdk_grasp_dry_run.py
```

这个脚本会调用 neck 低头视觉位，然后跑视觉检测、选择一个 `valid=True` 的塑料袋目标，并打印下一阶段准备发送的 SDK 运动计划。当前版本不会移动右臂，也不会闭合手掌。

如果脖子已经低头，只想调视觉：

```bash
python tools/run_sdk_grasp_dry_run.py --skip-neck
```

如果不想打开窗口：

```bash
python tools/run_sdk_grasp_dry_run.py --no-window
```

### 8. MPC 接口确认

```bash
python tools/debug_mpc_interfaces.py
```

这个脚本只查询 MPC 服务/话题，并短时间采样 `/DualArmMobile/currenState`、`/DualArmMobile/currentEEPose/FrameL`、`FrameR` 的消息结构，不会开启 MPC 模式，也不会发运动命令。

MPC 路线开始前必须先确认：

- 实际服务前缀：当前已确认为 `/wa`
- `points_seq_tracking` / `joints_seq_tracking` 服务类型存在
- `currentEEPose/FrameL` 和 `FrameR` 有数据
- `/DualArmMobile/currenState` 话题存在，类型是 `ocs2_msgs/mpc_target_trajectories`；当前 `rosbridge` 容器里 `rospack find ocs2_msgs` 失败，所以 `rostopic echo` 无法解析，维度仍待确认
- 腕力补偿话题当前不存在，导纳/腕力相关逻辑先不要启用

当前 MPC 决策：

- 暂时不用 `/wa/joints_seq_tracking`
- 暂时不依赖 `/DualArmMobile/currenState`
- 暂时不测试导纳/腕力路线
- 先测试 `/wa/points_seq_tracking` 任务空间路线

问题记录看：

```text
doc/MPC_ROUTE_ISSUES_20260721.md
```

### 9. MPC points_seq_tracking 干跑

```bash
python tools/run_mpc_points_dry_run.py
```

默认只读取 `/DualArmMobile/currentEEPose/FrameR`，构造 `/wa/points_seq_tracking` 请求并打印，不发送运动命令。

第一轮只看请求结构。确认后如果要做真机小幅测试，必须显式加 `--execute`，例如：

```bash
python tools/run_mpc_points_dry_run.py --dz 0.02 --execute
```

安全限制：

- `--execute` 位移单轴不能超过 `0.03m`
- `--execute` duration 不能小于 `5s`
- `--execute` weight 不能大于 `1.0`
- 不要直接把视觉相机坐标发给 MPC

### 10. 手眼对齐

正式手眼路线以课程手册为准：

```text
camera -> CAM2HEAD -> HEAD->BASE(tf) -> BASE object pose -> GRASP_OFFSET -> TCP/MPC target
```

详细对照看：

```text
handeye_calibration/HAND_EYE_ALIGNMENT_PLAN.md
```

手眼标定相关文件都放在 `handeye_calibration/`，和主程序分开：

```text
handeye_calibration/debug_handeye_sources.py
handeye_calibration/collect_cam2head_pairs.py
handeye_calibration/solve_cam2head.py
handeye_calibration/data/
handeye_calibration/calibration/
handeye_calibration/deprecated/
```

之前的 `deprecated/collect_handeye_pairs.py` / `deprecated/solve_handeye_alignment.py` 是固定脖子姿态下的实验简化方案，现在正式废弃，不再作为调试主线或交付路线。

下一步只做课程手册链路：

```text
确认 HEAD->BASE tf frame
确认是否已有 RealSense->HEAD 外参
没有外参则做 CAM2HEAD 标定
定义塑料袋 GRASP_OFFSET
用 robot_grasp/coordinate_utils.py 输出 TCP/MPC 目标
```

## 输出结果怎么看

`run_grasp.py` 和 `debug_vision_pipeline.py` 的核心输出是每个物体一条结果：

```text
#1 plastic_bag conf=0.820 qr= xyz=(120.5, 95.4, 820.0) status=valid
```

字段含义：

- `label`：YOLO 类别
- `conf` / `confidence`：检测置信度
- `qr` / `qr_text`：当前绑定的二维码文本；抓前可以为空
- `valid`：是否有可用 3D 坐标
- `x_mm/y_mm/z_mm`：相机坐标系下 3D 坐标，单位 mm
- `status`：深度/稳定状态

抓前推荐过滤条件：

```text
valid=True
confidence >= OBJECT_MIN_CONF
x_mm/y_mm/z_mm 非空
depth_age_ms <= DEPTH_MAX_AGE_MS
```

代码入口：

```python
from robot_grasp.grasp_flow import select_grasp_target

target = select_grasp_target(object_results)
```

抓后二维码绑定：

```python
from robot_grasp.grasp_flow import bind_qr_after_grasp

grasped_object = bind_qr_after_grasp(grasped_object, qr_text)
```

## 当前配置重点

配置集中在：

```text
robot_grasp/config.py
```

当前主要参数：

```python
WS_URL = "ws://192.168.20.98:9090"

YOLO_MODEL = "models/best.pt"
YOLO_TARGET_CLASSES = []
YOLO_CONF = 0.25
OBJECT_MIN_CONF = 0.30
YOLO_IMGSZ = 512
DETECT_EVERY_N_FRAMES = 1

ENABLE_DEPTH = True
DEPTH_TRANSPORT = "compressedDepth"
DEPTH_MAX_AGE_MS = 1000
DEPTH_DECODE_INTERVAL_SEC = 0.2

ENABLE_QR = False
QR_DECODE_INTERVAL_SEC = 0.8
QR_MAX_ROIS_PER_SCAN = 2
QR_MEMORY_TTL_SEC = 60.0
QR_RAW_RGB_THROTTLE_MS = 3000
```

`YOLO_TARGET_CLASSES = []` 表示不按类别名过滤，直接使用 `models/best.pt` 的输出。当前桌面阶段关闭 QR，因为业务已经改成抓后把塑料袋移动到相机前再近距离识别二维码。

当前话题：

```text
RGB compressed:
/zj_humanoid/sensor/realsense_head/color/image_raw/compressed

RGB raw:
/zj_humanoid/sensor/realsense_head/color/image_raw

Depth aligned compressedDepth:
/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth

Camera info:
/zj_humanoid/sensor/realsense_head/color/camera_info
```

## RGB 和深度怎么配合

当前不是精确分割，而是 YOLO bbox + aligned depth：

1. RGB 进 YOLO，得到 bbox 和中心点 `(u, v)`
2. 在 aligned depth 的同一 bbox 区域取有效深度
3. 当前策略取 bbox 下半部分深度中位数
4. 用相机内参反投影：

```text
z = depth_mm
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```

注意：当前第一版仍使用 bbox 取深度。塑料袋形状不规则，后续建议改为 segmentation mask，只统计 mask 内深度，避免桌面和背景污染。

## 性能指标

`analyze_perf.py` 会统计：

- `display_fps`：主循环显示帧率
- `rgb_rx_fps`：RGB 接收帧率
- `depth_msg_rx_fps`：compressedDepth 消息频率
- `depth_rx_fps`：成功解码的深度帧率
- `detect_fps`：YOLO 检测频率
- `infer_ms`：YOLO 推理耗时
- `depth_age_ms`：当前深度年龄
- `qr_decode_ms`：二维码解码耗时
- `raw_rgb_age_ms`：raw RGB 年龄
- `object_results`：每个物体的坐标和 QR 绑定结果

静态抓取参考阈值：

```text
display_fps >= 10
detect_fps >= 4
depth_rx_fps >= 1.5
depth_age_ms <= 1000ms
infer_ms <= 250ms
```

静态抓取不需要所有模块都实时到 30fps。更合理的是：RGB 保持流畅，深度低频但不太旧，目标停稳后采样多个深度点取中位数。

## 下一步接抓取

接下来代码上建议分三层：

```text
VisionPipeline
  -> object_results
  -> select_grasp_target()
  -> 头部 neck 固定到能看到桌面的视觉位
  -> 坐标转换 camera -> base/tcp/mpc
  -> SDK 或 MPC 运动客户端
  -> 抓取成功后移动到检查位
  -> 近距离 QR 检测
  -> bind_qr_after_grasp()
  -> 右臂 P3->P2->P1 安全回收
  -> 调用 /zj_humanoid/upperlimb/go_home/neck 复位头部
```

优先路线：

1. 先用 SDK `movej_by_path + movel + hand/joint_switch` 打通第一版抓取闭环
2. 再把同一套视觉输出接到 MPC
3. MPC 路线完整保留在 `doc/视觉抓取部署教程-MPC路线.md`

详细步骤看：

```text
doc/视觉抓取部署教程-SDK路线.md
doc/视觉抓取部署教程-MPC路线.md
```

## 重要踩坑

- 不要用 raw depth 作为主深度订阅；rosbridge 下 raw depth 会非常卡。当前使用 `compressedDepth`。
- RGB、depth、raw RGB 分开 client，避免大包互相堵塞。
- QR 不要为了桌面远距离扫码而无限外扩 ROI；后续塑料袋更应该用 mask。
- 抓前不要强制要求 `qr_text` 非空；现在是先抓后近距离识别。
- 右臂在 P3 桌面上方时，不要直接调用 `go_down/dual_arm`；必须先走 `P3 -> P2 -> P1` 回收路径。
- 脖子低头用于桌面视觉；手臂回收到 P1 后再调用 `/zj_humanoid/upperlimb/go_home/neck` 复位。放置阶段如果还需要摄像头，后面单独定义放置观察位。
- SDK 和 MPC 不要同时抢控制权；切换规则看部署教程。
- WA2 的 MPC 全身关节数是 23，不要沿用 WA1 的 20。
- 厂商候选 `CAM2HEAD` 已保存到 `handeye_calibration/calibration/cam2head_vendor_20260724.json`，但来源是别人标定结果，只能先 dry-run 验证，不能直接用于真实运动。
- 手动调用任何 SDK 服务前，先关闭 MPC：`rosservice call /wa/wa_hardware_interface/mpc_mode_setting "data: false"`。代码里的 `SDKMotionClient` 也会在 SDK 调用前自动尝试关闭 MPC。

## 备份说明

旧文档没有删除，已移动到：

```text
backup_old_docs_20260720/
```

根目录现在只保留继续开发需要的当前文档、代码、模型和调试脚本。
