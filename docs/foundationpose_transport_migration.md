# FoundationPose Transport Migration

FoundationPose 原生 conda 环境已经验证可跑。该路线已经作为运输阶段默认箱体识别方案接入；原 FastSAM + depth-rim 方案保留为 legacy 回退。

## 当前接入状态

```text
third_party/foundationpose_crate/                       项目内 FoundationPose 副本
apps/transport/run_foundationpose_box_grasp_point.py    FoundationPose 识别适配入口
apps/transport/run_transport_flow.py                    默认 backend=foundationpose
apps/transport/legacy/run_box_grasp_point_fastsam.py    原运输识别备份
apps/transport/run_box_grasp_point.py                   原脚本仍保留，可直接回退测试
```

有限帧检测结束后，项目内 `third_party/foundationpose_crate/run_live.py` 默认保留最后一次识别结果，不再把 `latest_grasp_points.json` 覆盖为 `PROCESS_NOT_RUNNING`。如需恢复原行为，可显式传 `--mark-not-running-on-exit`。

## 当前验证结论

已在 `foundationpose_detect` 环境验证：

```text
torch 2.11.0+cu128
CUDA available: True
GPU: NVIDIA GeForce RTX 5060 Laptop GPU
pytorch3d: ok
nvdiffrast: ok
FoundationPose estimater: ok
mycpp Python 3.10 extension: ok
run_single.py offline inference: ok
run_live.py rosbridge RGB-D live inference: ok
```

同时验证原项目抓取/运输依赖没有明显冲突：

```text
YOLO plastic bag model: ok
FastSAM model: ok
OCR ONNX CUDA provider: ok
WeChat QR OpenCV backend: ok
apps/grasp/run_grasp_flow.py --help: ok
apps/transport/run_box_grasp_point.py --help: ok
apps/place/run_shelf_box_target.py --help: ok
python -m compileall apps robot_grasp tools: ok
```

结论：FoundationPose 依赖可以并入现有 `detect` 环境，但在正式合并前先保持独立清单 `requirements-foundationpose.txt`。

## 迁移目标

把外部项目：

```text
/home/hmit/naviai/plastic_crate_dual_grasp_foundationpose_20260814/foundationpose_cad
```

逐步收敛进本项目运输阶段：

```text
apps/transport/
robot_grasp/vision/
robot_grasp/transforms/
models/
configs/
data/runtime/
```

迁移后，运输阶段应输出与当前流程一致的 BASE 坐标抓取目标，而不是只输出相机光学坐标。

## 推荐目录映射

```text
foundationpose_cad/run_live.py
-> apps/transport/run_foundationpose_box_grasp_point.py

foundationpose_cad/run_single.py
-> tools/debug/debug_foundationpose_single.py

foundationpose_cad/dual_grasp_points.py
-> robot_grasp/vision/foundationpose_grasp_points.py

foundationpose_cad/make_bootstrap_mask.py
-> robot_grasp/vision/foundationpose_mask.py

foundationpose_cad/assets/plastic_crate_m.obj
-> models/foundationpose/plastic_crate_m.obj

foundationpose_cad/assets/crate_metadata.json
-> configs/transport_foundationpose_crate.json

foundationpose_cad/vendor/FoundationPose/
-> third_party/FoundationPose/ 或保留外部路径直到迁移稳定
```

当前已经把 `vendor/FoundationPose` 复制到 `third_party/foundationpose_crate/vendor/FoundationPose/`，包括权重和 Python 3.10 `mycpp` 编译产物。后续如果仓库体积压力较大，再单独评估是否改成外部依赖下载。

## 阶段 1：只读入口接入（已完成）

新增一个运输识别入口：

```bash
python apps/transport/run_foundationpose_box_grasp_point.py \
  --ws-url ws://192.168.20.102:9091 \
  --show-window
```

要求：

```text
订阅 RGB-D
默认控制 MPC neck 低头/抬头，可通过 --skip-neck-down/--skip-neck-home 关闭
不发 MPC
输出 camera optical frame 下的箱体 pose 和左右抓取点
保存 annotated/debug 图
```

这一阶段只验证算法稳定性，不接机器人动作。

## 阶段 2：接现有 CAM2HEAD + TF 转换（已接入主流程）

FoundationPose live 输出仍是相机光学坐标，不能直接发 MPC。

需要复用当前转换链：

```text
camera optical point
-> CAM2HEAD
-> sampled TF: BASE -> HEAD
-> BASE point
```

输出文件应从：

```text
data/runtime/transport_foundationpose_box_latest.json
```

转换为：

```text
data/runtime/transport_box_grasp_target_latest.json
```

并保持与现有 `apps/transport/lock_box_grasp_target.py` 兼容。

## 阶段 3：与现有运输流程并行切换（已完成）

当前运输流程命令：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --execute
```

显式回退旧 FastSAM：

```bash
python apps/transport/run_transport_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --backend fastsam \
  --geometry depth-rim \
  --rim-fit-mode side-mid \
  --execute
```

保留 `fastsam` 作为回退：

```text
foundationpose: CAD 6D pose 主方案
fastsam: 当前稳定可用方案
color: debug/兜底方案
```

## 阶段 4：动作链路接入

FoundationPose 提供左右抓取点后，动作仍沿用当前双手运输设计：

```text
低头识别
-> camera 坐标转 BASE
-> transport_home_dual
-> transport_pregrasp_dual
-> 后续双手 grasp/搬运动作
```

注意：

```text
1. 双手抓取不能直接复用塑料袋单手抓取参数。
2. 双手必须同步规划，避免一只手先接触盒子把目标推走。
3. FoundationPose 的左右点是短边顶沿点，需要现场确认与手掌实际接触位置一致。
4. 如果抓取点被遮挡，当前外部项目会输出 occlusion 状态；MPC 动作前必须检查 allowed/valid。
```

## 编译和安装注意事项

完整依赖见：

```text
requirements-foundationpose.txt
```

关键约束：

```text
CUDA 12.8 不能使用 GCC 14 编译 nvdiffrast/pytorch3d
必须安装 GCC/GXX 13
必须安装 cuda-libraries-dev，否则会缺 cusparse.h
mycpp 必须重新编译 Python 3.10 版本
```

已验证的关键命令：

```bash
conda activate foundationpose_detect
export CUDA_HOME="$CONDA_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST="12.0"

pip install --no-build-isolation git+https://github.com/NVlabs/nvdiffrast.git
pip install --no-build-isolation git+https://github.com/facebookresearch/pytorch3d.git
```

## 当前风险

```text
1. FoundationPose 输出日志很多，正式接入时需要 quiet wrapper。
2. 首帧 mask 初始化仍依赖颜色/profile，现场需要固定箱子颜色或配置 profile。
3. CAD 模型和实际箱子尺寸必须一致，否则抓取点会系统性偏移。
4. vendor 代码体积大，提交前需要确认仓库体积可接受。
5. 当前 live 输出 grasp=INVALID 时，需要区分是遮挡、深度无效、mask 初始化错误，还是算法误差。
```

## 下一步建议

```text
1. 现场跑 apps/transport/run_transport_flow.py --backend foundationpose --execute。
2. 检查 data/transport/foundationpose_box_grasp_debug_latest/latest.png 中 CAD 投影和左右抓取点。
3. 连续 5-10 次确认左右抓取点稳定。
4. 再接后续双手夹持、搬运和放置动作。
```
