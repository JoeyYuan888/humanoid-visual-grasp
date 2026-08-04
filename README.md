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

## 环境安装

推荐 Python 3.10。第一次在新电脑安装：

```bash
conda create -n detect python=3.10 -y
conda activate detect
```

使用国内 conda 源安装基础依赖：

```bash
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/free
conda config --add channels https://mirrors.tuna.tsinghua.edu.cn/anaconda/cloud/conda-forge
conda config --set show_channel_urls yes
conda install -n detect zbar -c conda-forge -y
```

先安装匹配本机 GPU 的 CUDA 版 PyTorch。

RTX 50 系列/RTX 5060 使用：

```bash
pip install -r requirements-torch-cu128.txt
```

其他显卡不要照抄上面的 cu128 文件，应按本机 GPU/驱动选择匹配的 CUDA 版 PyTorch。

然后安装项目通用依赖：

```bash
pip install --no-deps -r requirements.txt
```

安装项目依赖时必须使用 `--no-deps`，避免 `ultralytics` 把
`opencv-contrib-python` 替换成普通 `opencv-python`，也避免 CUDA 版 PyTorch
被错误依赖覆盖。

安装后确认 CUDA 可用：

```bash
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO CUDA')"
```

如果输出 `torch.cuda.is_available() == False`，先修 NVIDIA driver/CUDA 环境，不要改成 CPU 版。

## 快速运行

先在机器人端进入 `huimin1.4` Docker 容器，启动 MPC rosbridge 9091：

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

然后在电脑端使用 `detect` conda 环境运行完整流程：

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
