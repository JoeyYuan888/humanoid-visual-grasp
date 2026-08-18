# Configuration

This directory stores project-level parameter references.

Current scripts do not load these YAML files yet. Runtime paths remain:

```text
robot_grasp/common/config.py
data/poses/mpc_via*_pose_right.json
data/poses/mpc_qr_present_pose_right.json
data/poses/grasp_profile_*.json
data/calibration/cam2head_vendor_new_20260803.json
```

Use these YAML files as the human-readable reference before wiring a config
loader into runtime code.
