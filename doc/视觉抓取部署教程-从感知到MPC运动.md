# 视觉抓取部署教程索引

当前部署教程已经拆成两条路线，避免 SDK 与 MPC 操作混在一起：

```text
doc/视觉抓取部署教程-SDK路线.md
doc/视觉抓取部署教程-MPC路线.md
```

两份文档都保留共同内容：

```text
当前进度快照
安全原则
环境准备
视觉检测与深度定位
CAM2HEAD 手眼标定
抓取后近距离 QR 识别业务逻辑
调试与验证方法
```

区别：

```text
SDK 路线：使用 /zj_humanoid/upperlimb/* 和 /zj_humanoid/hand/*，适合先打通第一版真实抓取。
MPC 路线：使用 /wa/points_seq_tracking 等 MPC 接口，适合后续全身/多约束运动执行。
```

当前项目进度：

```text
推进到：Step 3 坐标系标定
当前子任务：头部 RealSense 的 CAM2HEAD 手眼标定
整体项目完成度：约 45%
当前卡点：缺 HEAD -> realsense_head_link 外参
```

建议阅读顺序：

```text
1. 若当前先继续 SDK 抓取闭环：读 doc/视觉抓取部署教程-SDK路线.md
2. 若当前继续 MPC 环境/控制验证：读 doc/视觉抓取部署教程-MPC路线.md
3. 手眼标定单独看：handeye_calibration/HAND_EYE_ALIGNMENT_PLAN.md
```
