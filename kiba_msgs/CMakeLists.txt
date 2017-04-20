cmake_minimum_required(VERSION 2.8.3)
project(kiba_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib_msgs
  trajectory_msgs)

add_message_files(FILES CubicSpline.msg)
add_service_files(FILES GenerateTrajectory.srv)
add_action_files(FILES Move.action)
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs trajectory_msgs)
