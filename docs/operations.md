# Operations Manual

## 1. Robot Side

In the `huimin1.4` Docker container:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

## 2. PC Side

```bash
conda activate detect
python apps/grasp/run_grasp_flow.py \
  --ws-url ws://192.168.20.102:9091 \
  --return-mode qr-present \
  --scan-qr-after-present \
  --qr-transport raw \
  --qr-raw-throttle-ms 3000 \
  --max-z 1.70 \
  --execute
```

## 3. Camera Check

If the camera image is missing:

```bash
python apps/grasp/test_yolo.py
```

The script checks the RealSense stream and calls restart if needed.

## 4. Current Runtime Files

```text
data/runtime/mpc_locked_target_latest.json
data/runtime/post_grasp_qr_latest.json
data/poses/mpc_via0_home_right.json
data/poses/mpc_via1_pose_right.json
data/poses/mpc_via2_pose_right.json
data/poses/mpc_via3_pose_right.json
data/poses/mpc_qr_present_pose_right.json
data/calibration/cam2head_vendor_new_20260803.json
```

Detailed parameter explanations are in `docs/grasp.md`.

Stage-specific details:

```text
docs/grasp.md                     塑料袋抓取
docs/post_grasp_identification.md 抓后 OCR/QR
docs/place.md                     货架放置
```
