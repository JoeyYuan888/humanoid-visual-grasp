# Humanoid Visual Grasping

这是项目根入口。只放新同事启动项目必须知道的信息：当前路线、最短运行命令、目录结构。

更完整的操作细节、参数来源、排障记录见：

```text
doc/视觉抓取完整技术文档.md
```

## 当前路线

当前只保留 MPC 主路线：

```text
头部相机识别塑料袋
-> 锁存 BASE 坐标
-> MPC 抓取
-> 举到头部相机前扫描 QR
-> 放回抓取点
-> via3->2->1->0 回收
```

SDK 路线、旧过程记录、旧参数文件已清理，避免误用。

## 快速运行

电脑端使用 `detect` conda 环境：

```bash
conda activate detect
python tools/run_mpc_full_grasp_flow.py \
  --ws-url ws://192.168.20.98:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --place-after-qr \
  --max-z 1.70 \
  --execute
```

视觉窗口检查：

```bash
python tools/test_yolo.py
```

## 当前固定配置

- CUDA is required. The project intentionally refuses CPU fallback for YOLO.
- 当前 CAM2HEAD 矩阵：
  `handeye_calibration/calibration/cam2head_vendor_new_20260803.json`
- 当前 ROS bridge:
  `ws://192.168.20.98:9091`
- 当前模型:
  `models/best.pt`

## 目录结构

```text
run_grasp.py              Main visual pipeline entry
robot_grasp/              Runtime visual grasping modules
tools/                    主流程、位姿采集、视觉辅助脚本
tools/debugs/             调试、dry-run、一次性验证脚本
handeye_calibration/      Camera-to-head calibration tooling and results
doc/                      Project documentation
models/                   YOLO weights used by the project
ros_pkgs/                 ROS packages copied to robot catkin workspace
  mpc_target/             MPC ROS message/service package
  mpc_hardware_interface/ MPC hardware interface service package
  ocs2_msgs/              OCS2 message package needed by MPC state topics
```

## 文档入口

不要从旧记录文档开始操作。文档阅读顺序：

```text
1. doc/视觉抓取完整技术文档.md
2. doc/README.md
```
