cmake_minimum_required(VERSION 3.0.2)
project(mpc_target)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  actionlib_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  AdmittancePointsSeqTrackingAction.msg
  AdmittancePointsSeqTrackingActionFeedback.msg
  AdmittancePointsSeqTrackingActionGoal.msg
  AdmittancePointsSeqTrackingActionResult.msg
  AdmittancePointsSeqTrackingFeedback.msg
  AdmittancePointsSeqTrackingGoal.msg
  AdmittancePointsSeqTrackingResult.msg
  HybridPointsSeqTrackingAction.msg
  HybridPointsSeqTrackingActionFeedback.msg
  HybridPointsSeqTrackingActionGoal.msg
  HybridPointsSeqTrackingActionResult.msg
  HybridPointsSeqTrackingFeedback.msg
  HybridPointsSeqTrackingGoal.msg
  HybridPointsSeqTrackingResult.msg
  JointsSeqTrackingAction.msg
  JointsSeqTrackingActionFeedback.msg
  JointsSeqTrackingActionGoal.msg
  JointsSeqTrackingActionResult.msg
  JointsSeqTrackingFeedback.msg
  JointsSeqTrackingGoal.msg
  JointsSeqTrackingResult.msg
  PointsSeqTrackingAction.msg
  PointsSeqTrackingActionFeedback.msg
  PointsSeqTrackingActionGoal.msg
  PointsSeqTrackingActionResult.msg
  PointsSeqTrackingFeedback.msg
  PointsSeqTrackingGoal.msg
  PointsSeqTrackingResult.msg
  PointsSeqTrackingWithJointsAction.msg
  PointsSeqTrackingWithJointsActionFeedback.msg
  PointsSeqTrackingWithJointsActionGoal.msg
  PointsSeqTrackingWithJointsActionResult.msg
  PointsSeqTrackingWithJointsFeedback.msg
  PointsSeqTrackingWithJointsGoal.msg
  PointsSeqTrackingWithJointsResult.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  HybirdPointsSeqTracking.srv
  JointsSeqTracking.srv
  MPCTargetLockWaistSetting.srv
  PointsSeqCollaborativeTracking.srv
  PointsSeqTracking.srv
  PointsSeqTrackingWithCollisionAvoidance.srv
  PointsSeqTrackingWithCollisionAvoidanceSingle.srv
  PointsSeqTrackingWithJoints.srv
  TeleInferenceReset.srv
  TrackingStop.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  actionlib_msgs
)
