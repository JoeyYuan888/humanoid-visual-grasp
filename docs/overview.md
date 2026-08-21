# Project Overview

当前项目最终按四个业务阶段组织：

```text
grasp -> transport -> navigation -> place
```

当前已经跑通并固定参数的阶段：

```text
grasp -> post-grasp OCR/QR -> place
```

`transport` 阶段正在开发；`navigation` 阶段暂未接入主流程。

## Recommended Reading Order

1. `docs/operations.md`

   现场运行手册。第一次操作先看这个。

2. `docs/grasp.md`

   塑料袋抓取阶段细节，只到右手抓住塑料袋为止。

3. `docs/post_grasp_identification.md`

   抓后 OCR/QR 识别。

4. `docs/place.md`

   货架箱子 AprilTag 定位、拉出、投放、推回。

5. `docs/flowchart.md`

   完整流程图。

6. `docs/troubleshooting.md`

   重大问题和解决方案。

7. `docs/transport.md`

   双手搬运盒子阶段设计边界。

8. `docs/navigation.md`

   导航阶段设计边界。

## 当前闭环

```text
塑料袋锁存 -> MPC 抓取 -> 抓后 OCR/QR -> 人工/导航到货架前 -> AprilTag 放置
```

## Code Entrypoints

日常运行入口统一放在：

```text
apps/grasp/
```

`tools/` 只保留工程工具目录，调试脚本在 `tools/debug/`；抓取业务入口和实现都在 `apps/grasp/`。

后续盒子搬运入口放在：

```text
apps/transport/
```

当前已接入运输阶段箱子识别/靠近/夹紧测试：

```bash
python apps/transport/run_transport_flow.py --ws-url ws://192.168.20.102:9091 --backend foundationpose --execute
```

## Reference Materials

```text
docs/reference/
```

该目录只用于追溯厂商资料、旧部署教程和历史记录，不作为日常操作入口。
