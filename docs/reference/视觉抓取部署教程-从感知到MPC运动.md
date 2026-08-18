# 视觉抓取部署教程索引

当前项目已经收敛为 **MPC-only** 路线。SDK 路线文档、SDK 专用脚本和旧 SDK 路径文件已经删除，避免新同事误用历史方案。

日常操作入口：

```text
docs/grasp.md
```

保留的部署参考：

```text
docs/reference/视觉抓取部署教程-MPC路线.md
```

完整技术文档已经覆盖当前可执行闭环：

```text
头部低头锁存塑料袋 BASE 坐标
-> MPC 右臂抓取
-> 移动到 QR 展示点
-> 头部相机扫码
-> 回抓取点放置
-> via3->2->1->0 回收
```

推荐阅读顺序：

```text
1. docs/overview.md
2. docs/grasp.md
3. docs/reference/视觉抓取部署教程-MPC路线.md
4. docs/reference/handeye_alignment_plan.md
5. docs/reference/WA型号-MPC使用接口文档-外部 副本.md
```
