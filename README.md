# Humanoid Visual Grasping

这是项目根入口。只放新同事启动项目必须知道的信息：当前路线、最短运行命令、目录结构。

更完整的操作细节、参数来源、排障记录见：

```text
docs/grasp.md
```

## 当前路线

当前项目按四个业务阶段规划：

```text
抓取 grasp -> 搬运 transport -> 导航 navigation -> 放置 place
```

当前已经实现并跑通的是 MPC 视觉抓取闭环，其中包含抓取后的相机前识别和放回抓取点：

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
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --qr-transport raw \
  --qr-raw-throttle-ms 3000 \
  --place-after-qr \
  --max-z 1.70 \
  --execute
```

默认不弹出抓后 OCR/QR 识别窗口，流程结束后会自动退出。需要人工查看扫码窗口时再加 `--show-qr-window`。

完整流程默认在低头锁存阶段启用轻量高光抑制，适配白色塑料袋过亮场景。需要回退原始图像检测时加：

```bash
--highlight-suppression none
```

完整流程默认启用右手指尖压力检查。若只是调试运动路径，可临时加
`--disable-pressure-checks`，正式抓取不要关闭。

视觉窗口检查：

```bash
python apps/grasp/test_yolo.py
```

盒子搬运第一步只读检测：

```bash
python apps/transport/run_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

该命令默认会用 MPC neck 低头识别，并在退出窗口后抬头。若只读当前头部姿态，追加
`--skip-neck-down --skip-neck-home`。

## 当前固定配置

- CUDA is required. The project intentionally refuses CPU fallback for YOLO.
- 当前 CAM2HEAD 矩阵：
  `data/calibration/cam2head_vendor_new_20260803.json`
- 当前 ROS bridge:
  `ws://192.168.20.102:9091`
- 当前模型:
  `models/yolo/best.pt`

## 目录结构

```text
robot_grasp/                 公共 Python 包
  vision/                    YOLO、深度、OCR/QR、相机守护
  mpc/                       MPC helper 预留
  hand/                      手掌控制、压力读取
  transforms/                手眼矩阵、TF、坐标转换
  common/                    config、logger、ROS client、utils

apps/                        可直接运行的业务主入口
  grasp/                     抓取阶段
  transport/                 双手搬运盒子阶段预留
  navigation/                导航阶段预留
  place/                     放置阶段预留
  run_full_delivery_flow.py  抓取/搬运 -> 导航 -> 放置总流程预留

tools/                       单项工具，不作为主业务入口
  debug/                     debug 脚本
  calibration/               手眼/工装标定工具
  capture/                   记录 via/pose/样本
  maintenance/               清理、检查、导出预留

ros_pkgs/                    需要拷到机器人容器编译的 ROS 包
configs/                     所有可调参数参考
data/                        运行时数据
models/                      模型文件
docs/                        文档
scripts/                     环境和现场辅助脚本
```

## 后续业务阶段规划

```text
apps/grasp/
  负责识别塑料袋、抓取、抓后 OCR/QR 识别、当前已验证的放回抓取点。

apps/transport/
  后续负责盒子识别、左右手抓取点估计、双手夹持和搬运姿态控制。
  不要把盒子搬运逻辑写进 apps/grasp/。
  当前已有只读抓取点识别入口：apps/transport/run_box_grasp_point.py。

apps/navigation/
  后续负责根据识别结果选择目的地、调用导航、判断到达状态。

apps/place/
  后续负责到达目标位置后的独立放置策略。当前放置动作仍在抓取闭环里，
  等导航接入后再拆成独立阶段。
```

原则：公共 ROS/视觉/手掌模块放 `robot_grasp/`；可复用工程工具放 `tools/`；
操作者每天要执行的入口放 `apps/`。

## 文档入口

不要从旧记录文档开始操作。文档阅读顺序：

```text
1. docs/overview.md
2. docs/operations.md
3. docs/grasp.md
4. docs/flowchart.md
5. docs/troubleshooting.md
```
