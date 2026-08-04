cmake_minimum_required(VERSION 3.0.2)
project(ocs2_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  constraint.msg
  controller_data.msg
  joint_acceleration_mapped_policy.msg
  joint_impedance_mapped_policy.msg
  lagrangian_metrics.msg
  mode_schedule.msg
  mpc_flattened_controller.msg
  mpc_input.msg
  mpc_observation.msg
  mpc_performance_indices.msg
  mpc_state.msg
  mpc_target_trajectories.msg
  multiplier.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  reset.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
