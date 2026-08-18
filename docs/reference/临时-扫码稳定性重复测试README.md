# 临时：扫码稳定性重复测试 README

用途：重复验证 QR 扫码稳定性。当前保留两个入口：

1. 轻量入口：5 张 raw，边采边扫，不自动跑离线重恢复。全流程默认使用这个入口。
2. 重模型入口：15 张 raw，在线失败后自动跑离线多帧恢复。仅用于测试阶段。

## 0. 前置

机器人 `huimin1.4` 容器里启动 rosbridge：

```bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

电脑端：

```bash
conda activate detect
cd /home/hmit/naviai/center
```

## 1. 跑到扫码点

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --max-z 1.70 \
  --execute
```

执行后机器人会停在 QR 展示点。扫码阶段会自动保存最近 5 张 raw：

```text
data/runtime/qr_multiframe_debug/latest_raw/raw/
```

扫码现在是边采集边在线解码：每拿到一张 raw 就立刻尝试扫码。在线命中后会立即停止继续采集。

全流程默认不自动调用离线重恢复。这个入口用于正常抓取流程。

## 2. 重模型测试入口

只在需要测试 QR 算法时使用。它会采 15 张，并在在线失败后自动调用离线多帧恢复：

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --qr-snapshot-attempts 15 \
  --auto-recover-qr-offline \
  --max-z 1.70 \
  --execute
```

重模型输出目录：

```text
data/runtime/qr_multiframe_debug/latest_recover/
```

离线恢复包含：

```text
QR 候选框裁剪
多 padding 解码
mean / median / sharpest / laplacian_weighted 多帧融合
x4 / x8 超分变体
zxing-cpp / qreader / pyzbar / OpenCV 解码
```

注意：`ChecksumError` 结果一律拒绝，宁可失败也不接受错误码。

## 3. 判断在线扫码结果

如果终端出现：

```text
[QR] <二维码内容>
```

或结果文件里有 `texts`：

```bash
cat data/runtime/post_grasp_qr_latest.json
```

说明本轮在线扫码成功。

## 4. 在线没扫出时，手动复跑离线补扫

机器人先保持在 QR 展示点，不要恢复。手动运行：

```bash
python tools/debug/debug_qr_multiframe_recover.py \
  --input-dir data/runtime/qr_multiframe_debug/latest_raw/raw \
  --output-dir data/runtime/qr_multiframe_debug/latest_recover \
  --count 5 \
  --fast \
  --no-wechat \
  --min-consensus 2
```

如果输出类似：

```text
[OK] qreader-crop: <二维码内容> ... count=...
```

说明离线补扫成功。结果会写入：

```text
data/runtime/post_grasp_qr_latest.json
data/runtime/qr_multiframe_debug/latest_recover/report.json
```

如果本轮是重模型入口采到的 15 张，则把 `--count 5` 改为：

```text
--count 15
```

## 5. 恢复机器人，准备下一轮

先在机器人容器里松手：

```bash
rosservice call /zj_humanoid/hand/joint_switch/right "{q: [-0.1, 0.05, 0.35, 0.35, 0.35, 0.35]}"
```

然后电脑端回 `via0`：

```bash
python apps/grasp/run_visual_grasp_test.py \
  --ws-url ws://192.168.20.102:9091 \
  --use-locked-target data/runtime/mpc_locked_target_latest.json \
  --via-file data/poses/mpc_via3_pose_right.json \
  --via-file data/poses/mpc_via2_pose_right.json \
  --via-file data/poses/mpc_via1_pose_right.json \
  --via-file data/poses/mpc_via0_home_right.json \
  --stop-at-last-via \
  --use-joints \
  --max-motion 2.0 \
  --max-z 1.70 \
  --duration 5.0 \
  --execute-delay 0 \
  --execute \
  --confirm-target
```

恢复完成后，回到第 1 步重复测试。

## 6. 如果在线、自动离线、手动离线都失败

不要覆盖当前数据，保留：

```text
data/runtime/qr_multiframe_debug/latest_raw/raw/
data/runtime/qr_multiframe_debug/latest_recover/report.json
```

然后检查：

1. QR 是否完整进入画面。
2. 是否明显反光、模糊、遮挡。
3. `latest_raw/raw/` 里 raw 图是否确实是本轮扫码点采集。
4. `latest_recover/report.json` 里是否有 `qrdet` 检测框但无解码。

如果 report 中只检测到很小的框，例如小于 `80x80 px`，优先调整 QR 展示点，让二维码更靠近相机或更正对相机。算法可以做多帧超分，但二维码太小时仍然不稳定。

## 7. 本轮记录建议

每轮记录三项：

```text
第 N 轮:
  在线扫码: 成功/失败
  离线补扫: 成功/失败/未运行
  QR 内容:
  备注: 姿态、距离、是否反光、是否抓稳
```
