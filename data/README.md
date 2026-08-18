# Runtime Data

This directory contains runtime state, validated poses, calibration outputs,
and small offline samples.

Required by the current grasp flow:

```text
runtime/mpc_locked_target_latest.json
runtime/post_grasp_qr_latest.json
poses/mpc_via0_home_right.json
poses/mpc_via1_pose_right.json
poses/mpc_via2_pose_right.json
poses/mpc_via3_pose_right.json
poses/mpc_qr_present_pose_right.json
poses/grasp_profile_legacy_no_orientation.json
poses/grasp_profile_tuned_with_orientation.json
poses/mpc_grasp_tuned_pose_right.json
calibration/cam2head_vendor_new_20260803.json
```

Directory policy:

```text
runtime/      latest lock, latest OCR/QR result, temporary QR/OCR images
poses/        taught MPC via points, QR presentation pose, grasp profiles
calibration/  current effective hand-eye and vendor-board calibration files
samples/      CSV logs and small offline samples
```

Large or long-term datasets should not be committed here.
