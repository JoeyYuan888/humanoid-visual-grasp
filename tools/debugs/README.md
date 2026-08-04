# Debug Tools

这个目录只放排障、验证、一次性调试脚本，不属于日常完整抓取主流程。

主流程入口保留在上一级 `tools/`：

```text
tools/run_mpc_full_grasp_flow.py
tools/run_mpc_perception_lock.py
tools/run_mpc_visual_grasp_test.py
tools/run_post_grasp_qr_scan.py
tools/capture_mpc_pose.py
tools/test_yolo.py
```

本目录脚本用途：

```text
analyze_perf.py              分析 grasp_data_*.csv 性能日志
debug_cuda_env.py            检查 CUDA/PyTorch 环境
debug_mpc_interfaces.py      检查 MPC 话题和服务
debug_mpc_neck_control.py    检查 MPC neck/waist 相关服务
debug_ros_streams.py         检查 rosbridge 图像流
debug_select_target.py       从 CSV 中验证目标选择逻辑
debug_vision_pipeline.py     单独调试视觉管线
download_dataset.py          数据集下载辅助
find_neck_srv_md5_candidate.py 查找 neck srv md5 候选
robot_camera_demo.py         相机显示旧 demo
run_mpc_points_dry_run.py    小位移 points_seq_tracking dry-run
test_qr.py                   旧 QR 调试窗口
test_topics.py               ROS topic 测试
```
