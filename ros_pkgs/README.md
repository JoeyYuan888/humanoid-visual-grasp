# ROS Packages

这里存放需要拷贝到机器人/rosbridge 容器里编译的 ROS package。

目标位置：

```text
/workspace/catkin_ws/mpc_ws/src/
```

当前源码包：

```text
mpc_target/
ocs2_msgs/
mpc_hardware_interface/
```

压缩包留档：

```text
archives/mpc_target_catkin_ready_20260728.zip
archives/ocs2_msgs_catkin_ready.zip
archives/mpc_hardware_interface_catkin_ready_20260728.zip
archives/mpc_hardware_interface.zip
```

机器人容器内编译：

```bash
source /opt/ros/noetic/setup.bash
cd /workspace/catkin_ws/mpc_ws
catkin_make
source devel/setup.bash
```
