# Post-Grasp Identification

本文档描述塑料袋抓起后的 OCR/QR 识别，不描述抓取动作和货架放置动作。

边界：

```text
input : 右手已经抓住塑料袋，机器人可运动到 QR 展示点
logic : QR 展示点 -> 采集 raw 快照 -> OCR/QR 解码 -> 保存结果
output: 识别结果保存到 data/runtime/post_grasp_qr_latest.json
```

## 日常入口

日常抓取流程中会自动调用抓后识别：

```bash
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --qr-transport raw \
  --qr-raw-throttle-ms 3000 \
  --max-z 1.70 \
  --execute
```

## 识别策略

默认采集 5 张 raw 快照：

```text
小模型 OCR
-> PP-OCRv4 fallback
-> 轻量 QR fallback
```

输出文件：

```text
data/runtime/post_grasp_qr_latest.json
```

raw 快照目录：

```text
data/runtime/qr_multiframe_debug/latest_raw/raw/
```

离线恢复调试：

```bash
python apps/grasp/run_post_grasp_qr_scan.py \
  --recover-offline \
  --raw-frame-dir data/runtime/qr_multiframe_debug/latest_raw/raw \
  --recover-output-dir data/runtime/qr_multiframe_debug/latest_recover
```

病区字典从配置读取：

```text
configs/ocr/ward_directory.json
```

不要在代码里新增硬编码病区兜底。

## 压力检查

识别前后都只做轻量压力确认：

```text
carry_pressure_threshold = 0.00
qr_pressure_threshold    = 0.00
```

压力不足时固定策略：

```text
不补夹
不改手掌 q
回 via1
打开手掌
重新低头锁存
最多重试一次
```

## 注意事项

```text
1. 识别结果只在终端打印和 JSON 保存，不需要弹窗。
2. 如果识别失败，优先检查 latest_raw/raw 下的原图。
3. 如果手机能扫但程序不能扫，先跑离线恢复调试，不要立即改机器人姿态。
4. 抓后识别不是货架放置；放置动作看 docs/place.md。
```
