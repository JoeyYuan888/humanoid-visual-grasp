# Debug Tools

这个目录只放排障、验证、一次性调试脚本，不属于日常完整抓取主流程。

日常主流程入口在 `apps/grasp/`：

```text
apps/grasp/run_grasp_flow.py
apps/grasp/run_perception_lock.py
apps/grasp/run_visual_grasp_test.py
apps/grasp/run_post_grasp_qr_scan.py
apps/grasp/test_yolo.py
```

主流程入口在 `apps/`，例如 `apps/grasp/run_grasp_flow.py`；`tools/` 只保留单项工具和 debug 工具。

本目录脚本用途：

```text
analyze_perf.py              分析 grasp_data_*.csv 性能日志
debug_cuda_env.py            检查 CUDA/PyTorch 环境
debug_mpc_interfaces.py      检查 MPC 话题和服务
debug_mpc_neck_control.py    检查 MPC neck/waist 相关服务
debug_ocr_image.py           本地图片 OCR 调试
debug_ocr_webcam.py          本地 USB 摄像头 OCR 调试
debug_ros_streams.py         检查 rosbridge 图像流
debug_select_target.py       从 CSV 中验证目标选择逻辑
debug_vision_pipeline.py     单独调试视觉管线
download_dataset.py          数据集下载辅助
find_neck_srv_md5_candidate.py 查找 neck srv md5 候选
label_ocr_dataset.py         半自动 OCR 数据标注
robot_camera_demo.py         相机显示旧 demo
run_mpc_points_dry_run.py    小位移 points_seq_tracking dry-run
test_mpc_point_joints.py     MPC 移动到绝对点位并读取各关节角度
test_qr.py                   旧 QR 调试窗口
test_topics.py               ROS topic 测试
```

OCR 标签裁剪调试示例：

```bash
python tools/debug/debug_ocr_image.py data/runtime/qr_multiframe_debug/latest_raw/raw \
  --label-crop \
  --no-window \
  --save-debug \
  --debug-output data/runtime/ocr_label_debug/latest \
  --upscale 4.0 \
  --max-label-candidates 1 \
  --label-expand-x 0.35 \
  --label-expand-y 0.50
```
