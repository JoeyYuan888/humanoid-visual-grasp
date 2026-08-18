# 塑料框双短边抓取点定位

本项目使用 RGB、对齐深度和塑料框 CAD 模型，在实时画面中估计塑料框位姿，并输出两条短边上沿中心的三维坐标及遮挡状态，供后续机器人抓取流程使用。

当前方案已经在项目现有 RTX 4090 环境和 rosbridge 实时数据上跑通。默认检测青蓝绿色塑料框；颜色可通过配置文件更换，不需要修改算法代码。

> 安装新设备时请先阅读 [INSTALL.md](INSTALL.md)。当前 Docker 镜像适用于已经验证的 RTX 40 系列环境，不能直接用于 RTX 50 系列或 ARM 架构 Jetson Orin。

## 1. 功能概览

- 从 rosbridge 同步接收彩色图和对齐到彩色相机的深度图。
- 首帧使用可配置 HSV 颜色区域生成初始化掩膜。
- 使用 FoundationPose 和塑料框 OBJ 模型完成 6D 位姿注册与逐帧跟踪。
- 根据 CAD 几何精确计算两条短边上沿的中心，不使用检测框中心代替。
- 逐帧核验抓取点附近是否仍属于塑料框颜色。
- 抓取点可见时，使用上沿局部的实测深度计算 XYZ。
- 抓取点被不同颜色的塑料袋遮挡时，利用同一条短边其余可见部分拟合上沿，并预测该抓取点的 XYZ。
- 每个抓取点独立输出坐标是否有效、是否被遮挡、坐标来源和拟合质量。
- 实时保存 JSON、纯数字文本和可视化图，方便机器人程序或其他项目接入。
- 支持宿主机窗口实时显示；按 `q` 或 `Ctrl+C` 可正常退出。

## 2. 当前适用范围

当前版本的目标是：画面中存在一个主要塑料框，通过颜色获得初始区域，再由 CAD 位姿持续跟踪并输出双抓取点。

已支持：

- 塑料框不在画面中心；
- 塑料框部分超出画面；
- 框内塑料袋或货物高于框口；
- 一个或两个理论抓取点被不同颜色的塑料袋遮挡；
- 切换塑料框颜色配置；
- RGB 与深度不在本机，通过 rosbridge 接收。

当前没有承诺支持：

- 同时识别并编号多个外观相同的塑料框；
- 一排、多层货架中自动选择指定塑料框；
- 跟踪彻底丢失后的自动全局重识别；
- 塑料袋与塑料框颜色非常接近时仅靠颜色判断遮挡；
- 输出机器人基座坐标。当前 XYZ 是相机光学坐标，机器人使用前仍需外参变换；
- 遮挡点直接执行抓取。被遮挡时会输出预测坐标，但默认闭锁机器人动作。

## 3. 系统流程

```text
rosbridge
  ├─ RGB 压缩图像
  └─ 对齐到 RGB 的深度图
          │
          ▼
  HSV 初始区域 ──> FoundationPose + CAD ──> 塑料框 6D 位姿
                                              │
                                              ▼
                                   两条短边上沿理论中心
                                              │
                        ┌─────────────────────┴─────────────────────┐
                        ▼                                           ▼
                 中心可见且颜色匹配                         中心被异色物体遮挡
                        │                                           │
                 局部实测深度 XYZ                         可见上沿拟合并预测 XYZ
                        │                                           │
                        └─────────────────────┬─────────────────────┘
                                              ▼
                                  坐标、遮挡状态、质量指标
```

## 4. 项目结构

```text
foundationpose_cad/
├── README.md                         # 本说明
├── INSTALL.md                        # RTX 40/50、Jetson Orin 安装教程
├── run_live.sh                       # Docker 实时运行入口
├── run_live.py                       # 实时 rosbridge、位姿和抓取点主程序
├── show_live.py                      # 宿主机实时显示窗口
├── run_offline.sh                    # 离线 FoundationPose/CAD 基础测试
├── run_single.py                     # 单组离线数据入口
├── dual_grasp_points.py              # 双短边抓取点和遮挡判断
├── make_bootstrap_mask.py            # 颜色初始化掩膜工具
├── config/
│   ├── bootstrap_mask.json           # HSV 颜色与形态学配置
│   ├── cam_K.txt                     # 当前相机内参矩阵
│   └── realsense_head_intrinsics.json
├── assets/
│   ├── plastic_crate_m.obj           # 运行使用的米制 CAD 网格
│   ├── plastic_neutral.mtl            # 中性材质；算法不依赖模型文件颜色
│   └── crate_metadata.json            # 尺寸、坐标轴、上沿几何和对称性
├── data/live_sample_001/             # 一组离线 RGB/深度/掩膜样本
├── vendor/FoundationPose/            # 固定版本的 FoundationPose 源码
│   └── weights/                      # FoundationPose 两个网络权重
└── outputs/live/                     # 实时结果，运行后生成
```

## 5. CAD 与抓取点定义

### 5.1 塑料框尺寸和坐标系

运行模型使用米为单位：

- 长：`0.410 m`
- 宽：`0.270 m`
- 高：`0.142 m`
- 原点：塑料框底部中心
- CAD `+X`：长边方向
- CAD `+Y`：短边方向
- CAD `+Z`：从框底指向框口

塑料框的短边是 `X = 常量` 的两面。因此两个短边上沿中心的 CAD 坐标为：

```text
(-0.200, 0.000, 0.142) m
(+0.200, 0.000, 0.142) m
```

这些点位于上沿带的中心，而不是悬空的框内中心。程序会围绕理论中心检查一段短边边沿，并从可见边沿获取或预测深度。

### 5.2 点的顺序

输出数组始终按图像横坐标排序：

1. `image_left`：画面中更靠左的抓取点；
2. `image_right`：画面中更靠右的抓取点。

不要用数组顺序推断 CAD 的 `+X/-X`。塑料框关于 Z 轴存在 180° 对称，位姿解可能在等价方向间切换；图像左右顺序是对机器人接口更稳定的定义。

## 6. 输入数据

默认 rosbridge 地址和话题：

```text
ws://192.168.20.102:9091
/zj_humanoid/sensor/realsense_head/color/image_raw/compressed
/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth
```

要求：

- RGB 分辨率：当前配置为 `1280 × 720`；
- 深度必须已经对齐到 RGB；
- 深度图为 `uint16` 毫米，程序乘以 `0.001` 转换为米；
- RGB 和深度时间戳差不超过 `10 ms`；
- 两者使用同一彩色相机光学坐标系；
- 当前内参：

```text
fx = 916.3634    fy = 917.2302
cx = 645.2561    cy = 375.6018
```

如果更换相机、分辨率或裁剪方式，必须重新填写 `config/cam_K.txt`，不能沿用旧内参。

## 7. 快速运行

### 7.1 前提

以下命令假设：

- 当前机器是已经部署好的 RTX 40 系列 x86_64 主机；
- Docker 和 NVIDIA Container Toolkit 可用；
- 项目目录包含模型权重和已经编译的 `mycpp` 扩展；
- 本机能访问 `192.168.20.102:9091`。

新机器请先按 [INSTALL.md](INSTALL.md) 安装。

### 7.2 检查 rosbridge

```bash
nc -vz -w 3 192.168.20.102 9091
```

看到 TCP 连接成功后再运行程序。该检查只说明端口可达，不代表 RGB、深度话题一定在发布。

### 7.3 无窗口运行

```bash
cd /home/yons/test_plastic_crate/foundationpose_cad
./run_live.sh
```

### 7.4 带实时窗口运行

首次在新路径运行时，建议显式指定宿主机 Python。窗口程序只需要 OpenCV 和 NumPy，不在 Docker 内打开 GUI。

```bash
cd /home/yons/test_plastic_crate/foundationpose_cad
export PLASTIC_CRATE_PYTHON=/path/to/viewer-venv/bin/python
./run_live.sh --show
```

退出方式：

- 窗口获得焦点后按 `q`；或
- 运行命令的终端按 `Ctrl+C`。

程序退出时会把实时接口置为无效状态，避免下游继续使用陈旧坐标。

### 7.5 自定义 rosbridge 或话题

```bash
./run_live.sh \
  --ws-url ws://192.168.20.102:9091 \
  --color-topic /zj_humanoid/sensor/realsense_head/color/image_raw/compressed \
  --depth-topic /zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth
```

### 7.6 限制帧数用于测试

```bash
./run_live.sh --max-frames 30 --save-every 1
```

### 7.7 离线基础测试

```bash
./run_offline.sh --iteration 5 --debug 2
```

离线脚本用于验证 FoundationPose、CAD、相机内参和权重能否正常工作。它不连接 rosbridge，也不完整执行当前实时版的双抓取点遮挡发布逻辑。

## 8. 更换塑料框颜色

颜色配置位于 `config/bootstrap_mask.json`。默认激活：

```json
{
  "active_profile": "cyan_blue_green"
}
```

文件中预置了青蓝绿、蓝、绿、黄、橙、红、品红等 HSV 配置。更换框体颜色时：

1. 用 OpenCV HSV 范围描述新颜色；
2. 在 `profiles` 中增加一个名称明确的配置；
3. 把 `active_profile` 改成该名称；
4. 先用现场画面检查掩膜，再运行实时跟踪。

示例结构：

```json
{
  "active_profile": "my_crate",
  "profiles": {
    "my_crate": {
      "hsv_ranges": [
        {"lower": [80, 60, 40], "upper": [105, 255, 255]}
      ]
    }
  }
}
```

说明：

- OpenCV 的 H 范围是 `0–179`，S/V 是 `0–255`；
- 红色跨越 H 的首尾，需要两个区间；
- 初始检测不再忽略画面上方 60%；
- `1000` 像素目前仅作为小区域告警参考，不是硬性拒绝阈值；
- 算法不读取 OBJ/MTL 中的颜色，模型材质只用于中性显示；
- 颜色范围过宽会把背景物体纳入初始化，过窄会导致塑料框边沿断裂。

如果现场存在面积更大的同色背景物体，应收紧 HSV、调整机位或增加独立实例检测器。当前初始化会优先使用主要匹配区域，不负责多实例语义识别。

## 9. 抓取点深度和遮挡判断

### 9.1 可见抓取点

当理论中心附近满足以下条件时，输出实测坐标：

- 投影位于图像内；
- 附近存在有效深度；
- 颜色与当前塑料框配置匹配；
- 深度与 CAD 上沿预期位置连续。

此时：

```text
source = measured
occluded = false
```

### 9.2 被塑料袋遮挡的抓取点

如果理论中心被不同颜色的塑料袋遮挡，程序不会把塑料袋表面的深度当成框边沿深度。它会沿同一条短边寻找仍然可见的框体像素，转换为相机点云，并拟合 CAD 上沿所在的几何关系。

预测必须同时满足默认质量门限：

- 可见内点数不少于 `20`；
- 可见边沿跨度不少于整条短边的 `20%`；
- 拟合 RMSE 不大于 `0.012 m`。

满足时：

```text
source = predicted_from_visible_rim
occluded = true
```

预测坐标表示理论抓取位置，不表示该位置物理上可以立即抓取。因此只要任一抓取点被遮挡，顶层 `robot_execution_allowed` 默认就是 `false`。

### 9.3 无法可靠预测

如果可见边沿太少、深度空洞、拟合误差过大或点在画面外，该点会标为无效。不要使用上一帧坐标替代当前无效结果控制机器人。

## 10. 输出接口

所有实时文件位于 `outputs/live/`。

### 10.1 `latest_grasp_points.json`

这是推荐的主接口。典型结构如下，具体诊断字段可能随版本增加：

```json
{
  "valid": true,
  "coordinate_valid": true,
  "coordinate_frame": "realsense_head_color_optical_frame",
  "point_order": ["image_left", "image_right"],
  "points_left_to_right": [
    {
      "image_slot": "left",
      "cad_short_edge": "-x",
      "point_camera_m": [-0.12, 0.03, 0.91],
      "coordinate_valid": true,
      "valid": true,
      "occluded": false,
      "occlusion_state": "visible",
      "source": "measured"
    },
    {
      "image_slot": "right",
      "cad_short_edge": "+x",
      "point_camera_m": [0.13, 0.03, 0.92],
      "coordinate_valid": true,
      "valid": true,
      "occluded": true,
      "occlusion_state": "occluded_by_non_crate_color",
      "source": "predicted_from_visible_rim"
    }
  ],
  "any_grasp_point_occluded": true,
  "all_grasp_points_clear": false,
  "robot_execution_allowed": false,
  "sequence": 42,
  "published_at_unix_sec": 0.0,
  "rgb_stamp": 0.0,
  "depth_stamp": 0.0,
  "rgb_depth_delta_ms": 1.2,
  "crate_color_profile": "cyan_blue_green"
}
```

上例为了便于阅读省略了像素位置、拟合统计、CAD 诊断等字段。程序写出的实际 JSON 会保留这些诊断信息。

字段语义：

| 字段 | 含义 |
|---|---|
| `point_camera_m` | 相机光学坐标系中的 `[X,Y,Z]`，单位米 |
| `coordinate_valid` / `valid` | 当前帧坐标是否通过所有门限 |
| `occluded` | `false` 为中心实际可见，`true` 为中心被遮挡但坐标可预测，`null` 为无法判断 |
| `source` | `measured`、`predicted_from_visible_rim` 或无效原因 |
| `robot_execution_allowed` | 两点均有效且均未遮挡时才为 `true` |

### 10.2 纯数字接口

为便于非 Python 程序读取，还会写出：

- `latest_grasp_points.txt`：`2 × 3`，每行 `X Y Z`；
- `latest_grasp_occlusion.txt`：两个值，依次对应左右点；
- `latest_grasp_points_with_occlusion.txt`：`2 × 4`，每行 `X Y Z occluded`。

遮挡值定义：

```text
0  = 实际可见
1  = 实际被遮挡，XYZ 为预测值
-1 = 当前点无效或遮挡状态未知
```

无效 XYZ 写为 `NaN`。下游必须先检查有效性或 `NaN`，再使用坐标。

### 10.3 可视化和诊断

- `latest.png`：实时叠加图，`show_live.py` 会持续读取它；
- `last_valid_grasp_points_DIAGNOSTIC_ONLY.json`：最后一次有效结果，仅供排错，不得作为机器人实时控制输入；
- 其他调试图和位姿文件：由调试级别及保存间隔决定。

文件采用临时文件后原子替换的方式更新。下游仍应把每次读取视为一份完整快照，不要分别读取一半新一半旧的数据。

## 11. 接入机器人前必须完成的工作

当前坐标位于 RealSense 彩色相机光学坐标系：

```text
+X 向图像右侧
+Y 向图像下方
+Z 从相机向前
```

机器人控制通常需要基座或末端坐标。设相机到机器人基座的齐次变换为 `T_base_camera`，则：

```text
p_base = T_base_camera × p_camera
```

正式执行抓取前至少应增加：

1. 相机到机器人基座的手眼/外参标定；
2. 坐标时间戳与机器人状态同步；
3. 工作空间、碰撞和关节限位检查；
4. 对 `coordinate_valid`、`occluded` 和 `robot_execution_allowed` 的硬闭锁；
5. 连续多帧稳定性门限；
6. 末端抓具宽度、姿态和进退路径规划。

## 12. 常用参数

查看实时脚本的完整参数：

```bash
./run_live.sh --help
```

常用项：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `--ws-url` | `ws://192.168.20.102:9091` | rosbridge WebSocket 地址 |
| `--register-iteration` | `5` | 首次 FoundationPose 注册迭代数 |
| `--track-iteration` | `2` | 后续逐帧跟踪迭代数 |
| `--max-frames` | `0` | `0` 为持续运行，正数用于测试 |
| `--save-every` | `10` | 每隔多少帧保存调试结果 |
| `--grasp-depth-tolerance` | `0.08` | CAD 预期深度附近的容差 |
| `--grasp-center-color-min-ratio` | `0.25` | 中心邻域颜色命中比例门限 |
| `--grasp-prediction-min-inliers` | `20` | 遮挡预测最少边沿内点 |
| `--grasp-prediction-min-edge-span` | `0.20` | 遮挡预测最小可见跨度 |
| `--grasp-prediction-max-rmse` | `0.012` | 遮挡预测最大拟合误差 |

门限应使用现场录制数据调节。降低门限可能增加可用帧，也可能把塑料袋或背景误当成边沿；不要只根据单帧视觉效果放宽。

## 13. 常见问题

### 窗口提示 `QFontDatabase: Cannot find font directory`

这通常是 Qt 字体警告，不是核心算法崩溃原因。当前推荐的 `--show` 模式在宿主机显示 `latest.png`，容器本身不需要 Qt 窗口。

### 蓝色/青色检测区域不跟随塑料框

检查：

- `active_profile` 是否匹配实际颜色；
- HSV 范围是否把背景也选中；
- 是否正在查看过期的 `latest.png`；
- FoundationPose 是否已经跟踪失败。

当前版本不是 YOLO 多实例检测器。完全丢失后请停止并重新启动，或在后续版本中加入独立重识别模块。

### 画面卡住或无法停止

优先使用：

```bash
./run_live.sh --show
```

窗口按 `q`，终端按 `Ctrl+C`。不要在容器内部直接依赖 OpenCV/Qt 弹窗。若窗口画面不更新，检查主程序终端是否仍在收到 RGB/深度帧，以及 `outputs/live/latest.png` 的修改时间。

### 输出在停止后变成 `NaN` 和 `-1`

这是预期的失效闭锁行为。它防止机器人误用程序停止前的最后一个坐标。

### 两点位置看起来交换了

接口顺序是图像左、图像右，不是固定 CAD `-X/+X`。这是为处理塑料框 180° 对称而有意采用的规则。

### 抓取点被遮挡但仍有 XYZ

检查 `occluded` 和 `source`。`occluded=true` 时 XYZ 是由可见边沿预测的理论位置，不代表抓手能够穿过遮挡物；此时 `robot_execution_allowed` 应为 `false`。

### RTX 50 报 `no kernel image is available` 或扩展加载失败

不要继续使用当前 CUDA 12.1 镜像或 x86 预编译扩展。按 [INSTALL.md](INSTALL.md) 的 RTX 50 章节使用 CUDA 12.8 及对应 PyTorch，从源码重编译 nvdiffrast、PyTorch3D 和 `mycpp`。

### Jetson Orin 无法加载 `.so`

项目现有 `.so` 是 `x86_64 + Python 3.8`，不能在 `aarch64` 上使用。必须按 [INSTALL.md](INSTALL.md) 在 Orin 本机重新编译。

## 14. 安全说明

- 视觉输出只能作为机器人系统的一个输入，不应绕过碰撞检测和安全控制器。
- 被遮挡点即使坐标预测有效，也默认禁止直接执行。
- 深度相机在反光、透明、强光和物体边缘处可能产生空洞或跳变。
- 更换塑料框尺寸或 CAD 模型后，必须同步修改 `crate_metadata.json` 并重新验证抓取点。
- 更换相机、分辨率或对齐方式后，必须重新标定内参和机器人外参。
- 正式场地应录制包含远近距离、各层货架、部分遮挡和满载塑料袋的回归数据。

## 15. 上游项目与版本固定

核心位姿算法来自 [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)。本项目保留了固定的 vendor 源码、模型权重和本地扩展，以便复现实验结果。安装到新 GPU 架构时，原生扩展必须针对目标 CUDA、PyTorch、Python 和 CPU 架构重新编译。

当前随项目保存的两组权重应位于：

```text
vendor/FoundationPose/weights/2023-10-28-18-33-37/model_best.pth
vendor/FoundationPose/weights/2024-01-11-20-02-45/model_best.pth
```

部署时不要只复制 Python 文件而遗漏 `assets/`、`config/` 和包含权重的 `vendor/FoundationPose/`。
