# Project Overview

当前项目最终按四个业务阶段组织：

```text
grasp -> transport -> navigation -> place
```

当前已跑通的是 `grasp` 阶段，其中包含抓取后的 OCR/QR 识别和临时放回抓取点。

## Recommended Reading Order

1. `docs/operations.md`

   现场运行手册。第一次操作先看这个。

2. `docs/grasp.md`

   当前已跑通的 MPC 视觉抓取闭环细节。

3. `docs/flowchart.md`

   完整流程图。

4. `docs/troubleshooting.md`

   重大问题和解决方案。

5. `docs/transport.md`

   后续双手搬运盒子阶段设计边界。

6. `docs/navigation.md` / `docs/place.md`

   后续导航和独立放置阶段设计边界。

## 当前闭环

```text
识别塑料袋 -> MPC 抓取 -> 举到头部相机前扫码 -> 放回抓取点 -> via3->2->1->0 回收
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

当前已接入第一步只读检测：

```bash
python apps/transport/run_box_grasp_point.py --ws-url ws://192.168.20.102:9091 --show-window
```

## Reference Materials

```text
docs/reference/
```

该目录只用于追溯厂商资料、旧部署教程和历史记录，不作为日常操作入口。
