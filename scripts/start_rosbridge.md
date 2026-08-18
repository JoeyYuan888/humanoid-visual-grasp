# Start MPC Rosbridge

Run inside the robot `huimin1.4` container:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/mpc_ws/devel/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```

The PC side default URL is:

```text
ws://192.168.20.102:9091
```
