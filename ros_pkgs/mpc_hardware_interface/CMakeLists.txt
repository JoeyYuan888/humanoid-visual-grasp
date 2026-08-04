cmake_minimum_required(VERSION 3.0.2)
project(mpc_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  Joints.msg
  UplimbState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  MoveJ.srv
  MPCNeckJointMove.srv
  MPCWaistLockSetting.srv
  Servo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)
