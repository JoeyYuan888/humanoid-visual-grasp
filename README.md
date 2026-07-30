# Humanoid Visual Grasping

Head RealSense based visual grasping project for a humanoid robot. The current
pipeline detects plastic bags, computes 3D target coordinates, locks the target
in robot BASE coordinates, and prepares SDK/MPC grasp execution paths.

## Quick Start

Use the `detect` conda environment:

```bash
conda activate detect
python run_grasp.py
```

MPC perception lock flow:

```bash
python tools/run_mpc_perception_lock.py --ws-url ws://192.168.20.98:9091
```

## Current Notes

- CUDA is required. The project intentionally refuses CPU fallback for YOLO.
- Current CAM2HEAD result:
  `handeye_calibration/calibration/cam2head_vendor_board_20260729_164716.json`
- Main project documentation:
  `doc/README.md`
- MPC route:
  `doc/视觉抓取部署教程-MPC路线.md`
- SDK route:
  `doc/视觉抓取部署教程-SDK路线.md`

## Layout

```text
run_grasp.py              Main visual pipeline entry
robot_grasp/              Runtime visual grasping modules
tools/                    Debug, dry-run, MPC and SDK helper scripts
handeye_calibration/      Camera-to-head calibration tooling and results
doc/                      Project documentation and route records
models/                   YOLO weights used by the project
mpc_target/               MPC ROS message/service package
mpc_hardware_interface/   MPC hardware interface message/service package
ocs2_msgs/                OCS2 message package needed by MPC state topics
```
