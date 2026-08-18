# 安装与部署教程

本文说明如何把塑料框双短边抓取点项目安装到以下设备：

- x86_64 + NVIDIA RTX 40 系列；
- x86_64 + NVIDIA RTX 50 系列；
- ARM64 + NVIDIA Jetson Orin。

三类设备的 CUDA 架构和二进制兼容性不同，不能共用同一个预编译环境。请先执行“选择部署路线”，再进入对应章节。

项目运行原理、输入输出和参数说明见 [README.md](README.md)。

## 1. 先选择部署路线

| 目标设备 | 推荐路线 | 当前状态 | 关键要求 |
|---|---|---|---|
| RTX 40，x86_64 | 现有 Docker 镜像 | RTX 4090 已验证 | Docker、NVIDIA Container Toolkit、宿主机显示环境 |
| RTX 50，x86_64 | CUDA 12.8 + PyTorch cu128 源码构建 | 给出完整构建步骤，需在目标卡验收 | 不使用当前 CUDA 12.1 镜像；重编译全部 CUDA/C++ 扩展 |
| Jetson Orin，aarch64 | JetPack 原生环境或 NVIDIA Jetson PyTorch 容器 | 需要在目标 Orin 验收 | ARM 本机编译；不能使用 x86 `.so`；建议 16 GB 以上内存 |

为什么必须分开：

- RTX 50 属于 Blackwell。NVIDIA 的兼容性说明要求应用包含兼容 PTX 或为新架构重新生成代码；CUDA 架构矩阵把 Blackwell 的初始工具链列为 CUDA 12.8。当前项目镜像基于 CUDA 12.1，不能保证其 CUDA 扩展可在 RTX 50 上运行。[Blackwell Compatibility Guide](https://docs.nvidia.com/cuda/archive/12.8.2/blackwell-compatibility-guide/index.html)、[CUDA Architecture Matrix](https://docs.nvidia.com/datacenter/tesla/drivers/cuda-toolkit-driver-and-architecture-matrix.html)
- PyTorch 从 2.7/cu128 开始正式加入 Blackwell 支持；本教程为 RTX 50 固定到 PyTorch 2.11/cu128，避免使用已经不适合 Blackwell 的旧轮子。[PyTorch 2.7](https://pytorch.org/blog/pytorch-2-7/)、[PyTorch Previous Versions](https://pytorch.org/get-started/previous-versions/)
- Jetson Orin 是 `aarch64`，而项目现有 `mycpp.cpython-38-x86_64-linux-gnu.so` 是 x86_64/Python 3.8 二进制，无法跨架构加载。

## 2. 所有设备通用的准备工作

### 2.1 推荐硬件

- NVIDIA GPU 显存或 Jetson 统一内存：建议至少 `8 GB`，生产使用建议 `16 GB` 以上；
- 系统内存：建议 `32 GB`；
- 磁盘：至少预留 `20 GB` 给镜像、编译缓存和调试输出；
- 与 rosbridge 所在设备网络互通；
- 显示窗口可选，无显示器时可以无窗口运行。

NVIDIA Isaac ROS 的 FoundationPose 文档给出的峰值 GPU 占用约为 7 GB，并建议至少 8 GB；Jetson Orin Nano 4 GB 不适合作为本项目部署目标。[Isaac ROS FoundationPose](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/)

### 2.2 完整复制项目

目标机目录名可以自定义，但必须完整复制以下内容：

```text
foundationpose_cad/
├── assets/
├── config/
├── data/
├── vendor/FoundationPose/            # 包含源码和 weights/
├── *.py
├── run_live.sh
└── run_offline.sh
```

建议从源机器执行：

```bash
rsync -a --info=progress2 \
  /home/yons/test_plastic_crate/foundationpose_cad/ \
  user@TARGET_HOST:/opt/test_plastic_crate/foundationpose_cad/
```

不要只复制 OBJ 和 Python 脚本。两组 FoundationPose 权重、vendor 源码、相机配置和 CAD 元数据都是运行依赖。

### 2.3 检查文件完整性

进入目标机项目目录：

```bash
export CRATE_PROJECT=/opt/test_plastic_crate/foundationpose_cad
cd "$CRATE_PROJECT"
```

检查关键文件：

```bash
test -s assets/plastic_crate_m.obj
test -s assets/crate_metadata.json
test -s config/cam_K.txt
test -s vendor/FoundationPose/weights/2023-10-28-18-33-37/model_best.pth
test -s vendor/FoundationPose/weights/2024-01-11-20-02-45/model_best.pth
```

当前交付文件的 SHA-256：

```text
b5ff9aa47c3204d2098407248c0aedc5c47d59a130dc26f96ae88cc3504386ab  assets/plastic_crate_m.obj
774700586ddc435d408fc01c9809c43e151232936369dfbea0f0f964ba471d60  vendor/FoundationPose/weights/2023-10-28-18-33-37/model_best.pth
9d10a2c59400428d0411371d8030d1e12c838898ade32cec168faf9f95a0b54a  vendor/FoundationPose/weights/2024-01-11-20-02-45/model_best.pth
```

可以逐项执行 `sha256sum 文件名` 对比。若未来有意更新模型或权重，应同时更新部署记录，不应继续拿这里的旧哈希判断。

### 2.4 检查 CPU 架构

```bash
uname -m
```

- `x86_64`：进入 RTX 40 或 RTX 50 章节；
- `aarch64`：进入 Jetson Orin 章节。

### 2.5 检查 rosbridge 网络

默认连接：

```text
ws://192.168.20.102:9091
```

检查端口：

```bash
nc -vz -w 3 192.168.20.102 9091
```

如果端口不可达，先处理 IP、路由、防火墙或 rosbridge 服务；安装 CUDA 不能解决网络问题。

默认话题：

```text
/zj_humanoid/sensor/realsense_head/color/image_raw/compressed
/zj_humanoid/sensor/realsense_head/aligned_depth_to_color/image_raw/compressedDepth
```

计算机通过 rosbridge 接收数据，本项目本身不要求安装完整 ROS。只有当你要在本机直接查看 ROS 话题或改成 ROS 节点接口时才需要额外安装 ROS。

## 3. 路线 A：RTX 40 系列 x86_64

这是当前最稳妥的部署路线。现有 `run_live.sh` 默认使用：

```text
shingarey/foundationpose_custom_cuda121:latest
```

项目已经在 RTX 4090 环境验证。其他 RTX 40 型号仍应完整跑一次离线和实时验收。

### 3.1 安装 NVIDIA 驱动

安装适合显卡和 Ubuntu 版本的 NVIDIA 驱动，然后重启。确认：

```bash
nvidia-smi
```

命令应显示 GPU 型号、驱动版本和空闲显存。Docker 路线不要求宿主机安装与镜像完全相同的 CUDA Toolkit，但驱动必须满足镜像 CUDA 运行时要求。

### 3.2 安装 Docker Engine

按 Docker 官方 Ubuntu 教程安装 Docker Engine：[Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)。

安装后验证：

```bash
sudo docker run --rm hello-world
```

如果希望当前用户免 `sudo` 使用 Docker，应按 Docker 官方 post-install 说明加入 `docker` 组，重新登录后生效。加入该组等价于授予较高的主机权限，只应对可信用户这样做。

### 3.3 安装 NVIDIA Container Toolkit

按 NVIDIA 官方教程配置软件源并安装 Toolkit：[NVIDIA Container Toolkit Install Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)。

安装完成后配置 Docker 并重启：

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

验证容器可以访问 GPU：

```bash
sudo docker run --rm --gpus all ubuntu:22.04 nvidia-smi
```

### 3.4 拉取 FoundationPose 镜像

```bash
sudo docker pull shingarey/foundationpose_custom_cuda121:latest
```

如果使用私有镜像仓库或已经构建了自己的兼容镜像，可在运行时设置：

```bash
export FOUNDATIONPOSE_IMAGE=registry.example.com/foundationpose:your-tag
```

### 3.5 创建宿主机显示环境

`--show` 会在宿主机运行轻量查看器，持续显示 `outputs/live/latest.png`。创建一个 Python 虚拟环境：

```bash
cd "$CRATE_PROJECT"
python3 -m venv .venv-viewer
source .venv-viewer/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy opencv-python
deactivate
```

设置查看器 Python：

```bash
export PLASTIC_CRATE_PYTHON="$CRATE_PROJECT/.venv-viewer/bin/python"
```

可以把这一行加入专用启动脚本。不要照搬原开发机 `/home/yons/...` 的 Python 路径。

### 3.6 准备输出目录

```bash
cd "$CRATE_PROJECT"
mkdir -p outputs/live
chmod u+rwx outputs/live
chmod u+x run_live.sh run_offline.sh
```

### 3.7 离线验收

```bash
cd "$CRATE_PROJECT"
./run_offline.sh --iteration 5 --debug 2
```

首次运行可能需要生成 CUDA/Warp 缓存，因此会比后续慢。离线脚本成功说明：

- Docker GPU 透传正常；
- 权重能加载；
- CAD 和样本能读取；
- FoundationPose 基础推理能够完成。

离线脚本不验证 rosbridge 和实时双抓取点输出。

### 3.8 实时验收

无窗口：

```bash
./run_live.sh --max-frames 30 --save-every 1
```

带窗口：

```bash
export PLASTIC_CRATE_PYTHON="$CRATE_PROJECT/.venv-viewer/bin/python"
./run_live.sh --show --max-frames 30 --save-every 1
```

确认以下文件持续更新：

```bash
ls -l outputs/live/latest.png \
      outputs/live/latest_grasp_points.json \
      outputs/live/latest_grasp_points_with_occlusion.txt
```

正式持续运行：

```bash
./run_live.sh --show
```

## 4. 路线 B：RTX 50 系列 x86_64

### 4.1 不要使用什么

不要直接复用：

- `shingarey/foundationpose_custom_cuda121:latest`；
- 项目中现有 `mycpp.cpython-38-x86_64-linux-gnu.so`；
- 为 RTX 40/旧 CUDA 编译的 nvdiffrast 或 PyTorch3D 缓存。

这些文件即使偶尔能够加载，也不能证明其自定义 CUDA kernel 已覆盖 RTX 50 架构。

### 4.2 推荐软件组合

本教程使用下列可复现组合：

```text
Ubuntu 22.04 x86_64
支持 Blackwell 的 NVIDIA 驱动
CUDA Toolkit 12.8
Python 3.11
PyTorch 2.11.0 + cu128
```

PyTorch 官方已经说明 2.12 开始弃用 cu128，并建议 Blackwell 转向 CUDA 13 及更高驱动。为了减少当前 FoundationPose 依赖的变动，本项目先固定 2.11/cu128；以后升级 CUDA 13 时应重新做全套回归测试。[PyTorch 2.12 Release](https://pytorch.org/blog/pytorch-2-12-release-blog/)

### 4.3 安装驱动和 CUDA 12.8

1. 安装 NVIDIA 官方支持 RTX 50/Blackwell 的驱动；
2. 安装 CUDA Toolkit 12.8；
3. 重启；
4. 验证：

```bash
nvidia-smi
/usr/local/cuda-12.8/bin/nvcc --version
```

CUDA 12.8 版本说明见 [CUDA Toolkit 12.8 Release Notes](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html)。不要只根据 `nvidia-smi` 顶部显示的 CUDA 数字判断 Toolkit 已安装；`nvcc --version` 才检查本机编译工具链。

### 4.4 创建 Conda 环境

先安装 Miniforge、Miniconda 或其他可信 Conda 发行版，然后：

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose"
conda env create -f environment.yml
conda activate foundationpose
python --version
```

预期 Python 为 3.11。若环境已经存在：

```bash
conda env update -n foundationpose -f environment.yml --prune
conda activate foundationpose
```

### 4.5 安装 Blackwell 兼容 PyTorch

```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install \
  torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 \
  --index-url https://download.pytorch.org/whl/cu128
```

验证：

```bash
python - <<'PY'
import torch
print('torch:', torch.__version__)
print('torch CUDA:', torch.version.cuda)
print('CUDA available:', torch.cuda.is_available())
print('GPU:', torch.cuda.get_device_name(0))
print('capability:', torch.cuda.get_device_capability(0))
PY
```

必须满足 `CUDA available: True`，GPU 名称为目标 RTX 50。PyTorch 安装命令以后可能更新，可在 [PyTorch Previous Versions](https://pytorch.org/get-started/previous-versions/) 核对 cu128 命令。

### 4.6 安装 Python 依赖

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose"
python -m pip install -r requirements.txt
python -m pip install websocket-client
```

`websocket-client` 是本项目实时 rosbridge 接口额外需要的依赖，不是 FoundationPose 原始 requirements 的一部分。

### 4.7 从源码编译 PyTorch3D 和 nvdiffrast

确保编译时使用 CUDA 12.8：

```bash
export CUDA_HOME=/usr/local/cuda-12.8
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
```

安装 PyTorch3D：

```bash
python -m pip install --no-build-isolation \
  'git+https://github.com/facebookresearch/pytorch3d.git'
```

安装 nvdiffrast：

```bash
python -m pip install --no-build-isolation \
  'git+https://github.com/NVlabs/nvdiffrast.git'
```

nvdiffrast 官方源码和构建说明见 [NVlabs/nvdiffrast](https://github.com/NVlabs/nvdiffrast)。如果 PyTorch3D 最新主分支与固定 PyTorch 不兼容，应按 PyTorch3D 官方兼容说明选择标签，并把所用提交记录到设备部署清单中。

### 4.8 重编译 FoundationPose `mycpp`

最简单的 Conda 路线：

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose"
bash build_all_conda.sh
```

该脚本会清理并重建 `mycpp/build`。完成后确认生成的是当前 Python ABI 和 x86_64 文件：

```bash
find mycpp/build -maxdepth 1 -name 'mycpp*.so' -print
file mycpp/build/mycpp*.so
```

不要把原先 Python 3.8 的 `.so` 手工改名为 Python 3.11；那不会改变二进制 ABI。

### 4.9 检查环境

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose"
python check_env.py
```

再做最小导入测试：

```bash
cd "$CRATE_PROJECT"
python - <<'PY'
import torch
import cv2
import websocket
import nvdiffrast.torch
import pytorch3d
print('GPU:', torch.cuda.get_device_name(0))
print('imports: OK')
PY
```

### 4.10 运行 RTX 50 原生环境

RTX 50 路线不使用 `run_live.sh` 的默认 CUDA 12.1 容器，直接在已激活的 Conda 环境运行：

先用离线样本验证当前原生环境：

```bash
cd "$CRATE_PROJECT"
conda activate foundationpose
python run_single.py --iteration 5 --debug 2
```

再进行实时测试：

```bash
cd "$CRATE_PROJECT"
conda activate foundationpose
python run_live.py --max-frames 30 --save-every 1
```

带窗口：

```bash
python run_live.py --show --max-frames 30 --save-every 1
```

如果目标机无桌面环境，去掉 `--show`，通过 `latest.png` 或远程日志验收。

### 4.11 RTX 50 必做验收

- 导入测试无 `undefined symbol`；
- 不出现 `no kernel image is available for execution on the device`；
- 离线样本成功；
- 30 帧实时测试成功；
- `latest_grasp_points.json` 持续更新；
- 可见点为 `measured/occluded=false`；
- 人工遮挡中心后为 `predicted_from_visible_rim/occluded=true` 或安全无效；
- `Ctrl+C` 后接口变为无效，进程确实退出；
- 连续运行至少 2 小时，显存没有持续增长。

## 5. 路线 C：Jetson Orin ARM64

### 5.1 先明确 Orin 的现实边界

当前项目的 Python 算法已经在 RTX 4090 跑通，但没有在本次交付中对具体 Orin 型号完成实机验收。下面提供的是正确的 ARM 原生构建路线，不是“复制现有镜像即可运行”的承诺。

建议设备优先级：

1. AGX Orin 32 GB / 64 GB；
2. Orin NX 16 GB；
3. Orin Nano 8 GB 仅适合降分辨率、降迭代数后评估；
4. Orin Nano 4 GB 不建议。

如果最终产品确定使用 Orin，生产路线可以评估 NVIDIA Isaac ROS 3.2 的 FoundationPose，它明确支持 Jetson Orin + JetPack 6.1/6.2；当前最新版 Isaac ROS 的平台范围已经变化，因此要固定到匹配 Orin 的归档版本。[Isaac ROS 3.2 Pose Estimation](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/index.html)、[Isaac ROS Pose Estimation GitHub](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation)

迁移到 Isaac ROS 会把位姿层改成 ROS 2 节点，本项目的 `dual_grasp_points.py`、CAD 抓取点定义和遮挡接口仍可保留，但需要写适配层；它不是当前 Python 程序的无修改替代品。

### 5.2 安装 JetPack

推荐基线：

```text
JetPack 6.2.1
Ubuntu 22.04
Jetson Linux 36.4.4
CUDA 12.6
TensorRT 10.3
cuDNN 9.3
```

版本信息与刷机入口见 [JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621)。刷机后检查：

```bash
uname -m
cat /etc/nv_tegra_release
nvcc --version
```

预期 `uname -m` 为 `aarch64`。Jetson 通常没有与桌面 RTX 相同的 `nvidia-smi`；使用：

```bash
tegrastats
```

观察 GPU、内存、温度和功耗。

### 5.3 选择原生环境或 NVIDIA 容器

有两种可行方式：

- 原生 Python 虚拟环境：方便使用本机窗口和设备资源，本节给出主要步骤；
- NVIDIA Jetson 兼容 PyTorch 容器：依赖更集中，适合制作固定镜像，但仍要在容器内重编译 ARM 扩展。

NVIDIA 官方提供 Jetson 的 PyTorch 安装说明和 JetPack/容器兼容矩阵：[Installing PyTorch for Jetson Platform](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)、[PyTorch for Jetson Release Notes](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)。不要从普通 PyPI 安装面向 x86/CUDA 的 `torch` 来替换 NVIDIA Jetson 构建。

### 5.4 安装系统编译依赖

```bash
sudo apt update
sudo apt install -y \
  python3-dev python3-pip python3-venv \
  build-essential cmake ninja-build git pkg-config \
  libeigen3-dev libboost-all-dev pybind11-dev \
  libgl1 libglib2.0-0 libegl1-mesa-dev
```

### 5.5 创建原生 Python 环境

JetPack 的 NVIDIA Python 包可能装在系统目录，因此使用 `--system-site-packages`：

```bash
cd "$CRATE_PROJECT"
python3 -m venv --system-site-packages .venv-orin
source .venv-orin/bin/activate
python -m pip install --upgrade pip setuptools wheel
```

### 5.6 安装 NVIDIA Jetson PyTorch

1. 在 NVIDIA Jetson PyTorch 兼容矩阵中选择与当前 JetPack 完全匹配的版本；
2. 按官方页面复制该版本的 ARM64 wheel 安装命令；
3. 不要使用 RTX 50 章节的 `download.pytorch.org/whl/cu128` 命令；
4. 不要在后续 `pip install` 时让 pip 覆盖这个 PyTorch。

安装后验证：

```bash
python - <<'PY'
import platform
import torch
print('machine:', platform.machine())
print('torch:', torch.__version__)
print('CUDA:', torch.version.cuda)
print('available:', torch.cuda.is_available())
print('device:', torch.cuda.get_device_name(0))
PY
```

必须看到 `machine: aarch64` 和 `available: True`。

### 5.7 安装 FoundationPose Python 依赖

不要直接执行会覆盖 NVIDIA PyTorch/NumPy 的盲目全升级。先安装非 Torch 依赖：

```bash
python -m pip install \
  scipy scikit-learn h5py joblib PyYAML ruamel.yaml \
  opencv-python imageio trimesh transformations \
  matplotlib pandas Pillow pyrender PyOpenGL PyOpenGL_accelerate \
  kornia omegaconf psutil tqdm warp-lang websocket-client
```

FoundationPose 的工具模块还会导入 Open3D。ARM64 可用轮子取决于所选 JetPack/Python 版本：

```bash
python -m pip install open3d
```

如果提示没有匹配的 ARM64 wheel，不要安装 x86 wheel；应从 Open3D 官方源码为 aarch64 构建，或改用已包含兼容 Open3D 的 NVIDIA 容器。把最终 Open3D 版本写入部署记录。

### 5.8 在 Orin 上编译 PyTorch3D 和 nvdiffrast

```bash
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
```

```bash
python -m pip install --no-build-isolation \
  'git+https://github.com/facebookresearch/pytorch3d.git'

python -m pip install --no-build-isolation \
  'git+https://github.com/NVlabs/nvdiffrast.git'
```

编译时间可能较长，建议启用足够的 swap，但不要把 swap 当成可运行显存。若编译进程被 OOM 杀死，降低并行编译数，例如：

```bash
export MAX_JOBS=2
```

然后重新执行失败的安装命令。

### 5.9 在 Orin 上重编译 `mycpp`

不要使用项目中已有 x86 `.so`。在 Orin 本机执行：

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose/mycpp"
rm -rf build
mkdir build
cd build
cmake .. \
  -DPYTHON_EXECUTABLE="$(command -v python)" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build . -j2
```

确认架构：

```bash
find . -maxdepth 1 -name 'mycpp*.so' -print
file mycpp*.so
```

输出必须包含 `ARM aarch64`，文件名中的 Python ABI 必须与当前环境一致。

### 5.10 Orin 环境检查

```bash
cd "$CRATE_PROJECT/vendor/FoundationPose"
python check_env.py
```

最小导入测试：

```bash
cd "$CRATE_PROJECT"
python - <<'PY'
import platform
import torch
import cv2
import websocket
import nvdiffrast.torch
import pytorch3d
print('machine:', platform.machine())
print('GPU:', torch.cuda.get_device_name(0))
print('imports: OK')
PY
```

### 5.11 Orin 运行

使用原生环境时不要调用默认 x86 Docker 镜像，直接运行 Python：

先做离线测试：

```bash
cd "$CRATE_PROJECT"
source .venv-orin/bin/activate
python run_single.py --iteration 5 --debug 2
```

离线成功后再连接实时数据：

```bash
cd "$CRATE_PROJECT"
source .venv-orin/bin/activate
python run_live.py \
  --register-iteration 5 \
  --track-iteration 2 \
  --max-frames 30 \
  --save-every 1
```

有桌面时可以追加 `--show`。没有桌面时查看保存的 `latest.png`。

如果速度或内存不足，按以下顺序优化并逐项复测精度：

1. 把 `--track-iteration` 从 2 降到 1；
2. 降低调试保存频率；
3. 用相机端或独立预处理降低 RGB/深度分辨率，同时重新标定并更新内参；
4. 使用 Orin 的高性能功耗模式，并用 `tegrastats` 监控温度和降频；
5. 再评估 TensorRT/Isaac ROS 迁移。

不同 Orin 型号的 `nvpmodel` 模式编号不一致。先查看设备支持的模式，再选择最高性能模式：

```bash
sudo nvpmodel -q
sudo jetson_clocks
```

不要从其他型号教程直接照抄 `nvpmodel -m 某个数字`。

### 5.12 可选：使用 NVIDIA PyTorch Jetson 容器

JetPack 6.2 对应的 NVIDIA PyTorch 容器版本必须根据官方兼容矩阵选择。例如 NVIDIA 25.04 PyTorch 容器属于支持 Jetson iGPU 的版本范围，具体标签和 JetPack 对应关系以官方矩阵为准。[NVIDIA PyTorch 25.04 Release Notes](https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel-25-04.html)、[NGC PyTorch Container](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch)

容器路线的原则：

1. 选与 JetPack 匹配的 `nvcr.io/nvidia/pytorch:<version>-py3`；
2. 挂载完整项目目录和输出目录；
3. 不替换容器自带的 NVIDIA PyTorch；
4. 在该 ARM 容器内安装剩余依赖；
5. 在该 ARM 容器内重编译 PyTorch3D、nvdiffrast 和 `mycpp`；
6. 把完成后的环境固化为自己的版本化镜像；
7. 启动脚本显式设置这个 ARM 镜像，不能再使用默认 x86 镜像。

不要使用早期 `l4t-pytorch` 标签猜测兼容性；JetPack、L4T 和容器标签必须按官方表严格对应。

### 5.13 Orin 必做验收

- `uname -m` 为 `aarch64`；
- PyTorch CUDA 可用；
- 所有 `.so` 经 `file` 检查为 aarch64；
- 离线样本能完成；
- 实时 30 帧能完成；
- RGB/深度同步没有持续超时；
- 双点可见、单点遮挡、双点遮挡分别符合接口定义；
- `Ctrl+C` 后进程退出且输出失效；
- 连续运行 2 小时，无 OOM、温度降频或持续内存增长；
- 在最终机位和最远塑料框距离上仍有足够深度精度。

## 6. 相机与现场配置

### 6.1 修改 rosbridge 地址

临时指定：

```bash
./run_live.sh --ws-url ws://NEW_HOST:NEW_PORT
```

原生 Python 环境：

```bash
python run_live.py --ws-url ws://NEW_HOST:NEW_PORT
```

建议部署时由专用启动脚本传参，不要为了换场地到处修改源码默认值。

### 6.2 修改相机话题

```bash
python run_live.py \
  --color-topic /your/color/compressed \
  --depth-topic /your/aligned_depth/compressedDepth
```

深度必须对齐到彩色图。未对齐的原始 depth 不能仅靠修改话题直接使用。

### 6.3 更新内参

`config/cam_K.txt` 是 3×3 相机矩阵：

```text
fx 0  cx
0  fy cy
0  0  1
```

出现以下任一变化都要重新获取内参：

- 更换相机；
- 更换 RGB 分辨率；
- 裁剪或缩放图像；
- 改用另一条相机 pipeline；
- 改变对齐输出尺寸。

### 6.4 更新颜色

编辑 `config/bootstrap_mask.json` 中的 `active_profile` 和 HSV 范围。颜色只用于框体区域初始化和抓取点可见性核验，不读取 OBJ 材质颜色。详细方法见 README 的“更换塑料框颜色”。

## 7. 启动、停止和服务化

### 7.1 推荐人工启动命令

RTX 40 Docker：

```bash
cd "$CRATE_PROJECT"
export PLASTIC_CRATE_PYTHON="$CRATE_PROJECT/.venv-viewer/bin/python"
./run_live.sh --show
```

RTX 50 Conda：

```bash
cd "$CRATE_PROJECT"
conda activate foundationpose
python run_live.py --show
```

Orin venv：

```bash
cd "$CRATE_PROJECT"
source .venv-orin/bin/activate
python run_live.py
```

### 7.2 正常停止

- GUI 窗口按 `q`；
- 或终端按 `Ctrl+C`；
- 等待程序写入失效状态并退出。

不要使用 `kill -9` 作为日常停止方式。强制杀死会绕过清理逻辑，使下游短时间看到最后一份旧文件。下游无论如何都应检查时间戳和有效标志。

### 7.3 做成系统服务之前

先在命令行连续稳定运行，再考虑 systemd 或容器编排。服务化时需要额外处理：

- 启动前等待网络和 rosbridge；
- 显式工作目录；
- 显式 Python/Conda 环境；
- GPU 和 Docker 权限；
- 日志轮转；
- 异常重启退避；
- 输出文件时间戳监控；
- 停止时给足 SIGINT/SIGTERM 清理时间。

生产服务不建议打开 GUI。

## 8. 完整验收清单

### 8.1 环境

- [ ] CPU 架构与部署路线一致；
- [ ] GPU 被 PyTorch 或 Docker 正确识别；
- [ ] 关键权重、CAD、配置完整；
- [ ] 原生扩展与 Python/CUDA/CPU 架构一致；
- [ ] rosbridge 端口可达；
- [ ] RGB 和对齐深度话题持续发布。

### 8.2 功能

- [ ] 塑料框不在画面中心时仍能初始化；
- [ ] 两个抓取点位于两条短边上沿中心；
- [ ] 点顺序始终为图像左、图像右；
- [ ] 可见点输出 `measured` 和 `occluded=false`；
- [ ] 中心被异色塑料袋遮挡时输出预测坐标和 `occluded=true`；
- [ ] 可见边沿不足时安全输出无效；
- [ ] 颜色配置切换后无需修改代码；
- [ ] 停止后输出 `NaN/-1`，旧坐标不再允许执行。

### 8.3 性能与现场

- [ ] 最终最远距离仍有足够像素和有效深度；
- [ ] 最终最近距离不会严重超出画面；
- [ ] 满载、空框、塑料袋高出框口均测试；
- [ ] 不同货架层、照明和局部遮挡均测试；
- [ ] 连续运行无显存/内存泄漏；
- [ ] 相机到机器人基座外参已经标定；
- [ ] 机器人侧执行闭锁已经接入。

## 9. 常见安装故障

### Docker 中看不到 GPU

检查顺序：

```bash
nvidia-smi
docker info
sudo docker run --rm --gpus all ubuntu:22.04 nvidia-smi
```

宿主机 `nvidia-smi` 失败先修驱动；宿主机成功而容器失败，检查 NVIDIA Container Toolkit 和 Docker runtime。

### `ModuleNotFoundError: websocket`

```bash
python -m pip install websocket-client
```

不要误装名称相近但不同的 `websocket` 包。

### `ModuleNotFoundError: mycpp`

检查：

- 是否在正确项目/vendor 路径运行；
- `mycpp/build` 是否存在 `.so`；
- `.so` 的 Python ABI 是否匹配；
- `file mycpp/build/mycpp*.so` 的 CPU 架构是否匹配。

重新构建，不要改文件名伪装 ABI。

### `undefined symbol`、nvdiffrast 或 PyTorch3D 导入失败

通常是 PyTorch、CUDA 或 C++ ABI 不一致。删除该扩展自己的构建缓存，在当前环境中重新编译。不要从另一台机器复制 `.so`。

### RTX 50 出现 `no kernel image is available`

说明扩展没有为目标 Blackwell 架构生成可执行代码。确认使用 CUDA 12.8+/Blackwell PyTorch，并从源码重编译所有 CUDA 扩展。不要退回当前 CUDA 12.1 镜像碰碰运气。

### Orin 安装不到 Open3D/PyTorch3D

普通 PyPI 不保证每个 JetPack/Python 组合都有 aarch64 wheel。优先选择 NVIDIA 兼容容器；否则从官方源码在 Orin 编译，并固定提交。严禁安装 x86 wheel。

### 编译时被系统杀死

通常是内存不足。降低并行度：

```bash
export MAX_JOBS=2
```

`cmake --build` 使用 `-j2` 或 `-j1`。Orin 可配置临时 swap 帮助编译，但推理时仍需要真实可用内存。

### rosbridge 可连接但一直没有结果

检查：

- 话题名称；
- 压缩消息类型；
- RGB 和深度时间戳差；
- 深度是否对齐到 RGB；
- 相机内参与分辨率；
- 深度单位是否为毫米。

### GUI 无法打开

无桌面或 SSH 无图形转发时去掉 `--show`。Docker 路线的窗口在宿主机运行，所以宿主机查看器环境必须安装 OpenCV/NumPy，并设置正确的 `PLASTIC_CRATE_PYTHON`。

## 10. 版本记录建议

每台部署设备应保存一份机器可读或文本清单，至少包含：

```text
设备型号
CPU 架构
Ubuntu / JetPack 版本
NVIDIA 驱动版本
CUDA Toolkit 版本
PyTorch / torchvision 版本
PyTorch3D 提交
nvdiffrast 提交
FoundationPose vendor 提交
mycpp 文件名和 SHA-256
项目 Git 提交或交付日期
CAD 和权重 SHA-256
相机序列号、分辨率、内参版本
实测帧率、显存/内存峰值
验收数据集和验收结果
```

本项目 vendor 的 FoundationPose 固定提交为：

```text
a1b694b83e633c2cb6115b9063d940a687759392
```

现有开发机的 `mycpp` 哈希仅用于识别原 RTX 40/x86/Python 3.8 产物，不能作为 RTX 50 或 Orin 的目标哈希：

```text
a327c08cb2d6b831f4c906a6f92a823dfb261bc409895beeafb9781a3d9c4e66
```

## 11. 官方参考资料

- [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose)
- [NVIDIA Blackwell Compatibility Guide](https://docs.nvidia.com/cuda/archive/12.8.2/blackwell-compatibility-guide/index.html)
- [CUDA Toolkit Driver and Architecture Matrix](https://docs.nvidia.com/datacenter/tesla/drivers/cuda-toolkit-driver-and-architecture-matrix.html)
- [PyTorch Previous Versions](https://pytorch.org/get-started/previous-versions/)
- [Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [JetPack 6.2.1](https://developer.nvidia.com/embedded/jetpack-sdk-621)
- [Installing PyTorch for Jetson](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [PyTorch for Jetson Compatibility](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)
- [Isaac ROS FoundationPose 3.2 for Orin](https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_pose_estimation/index.html)
- [NVlabs/nvdiffrast](https://github.com/NVlabs/nvdiffrast)

安装教程中的第三方版本会随时间变化。新部署时应保留本教程的架构原则，并用上述官方兼容矩阵核对驱动、CUDA、PyTorch、JetPack 和容器标签。
