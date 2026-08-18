# Capture Tools

Pose and joint-state capture tools live here.

Run:

```bash
python tools/capture/capture_mpc_pose.py ...
```

Capture dual-arm MPC pose for transport:

```bash
python tools/capture/capture_dual_mpc_pose.py \
  --ws-url ws://192.168.20.102:9091 \
  --name transport_home \
  --include-joints \
  --output data/poses/transport/transport_home_dual.json
```
