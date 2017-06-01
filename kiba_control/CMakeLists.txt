cmake_minimum_required(VERSION 2.8.3)
project(kiba_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp ros_control_boilerplate interactive_markers geometry_msgs sensor_msgs control_msgs kdl_parser tf2_ros tf2_geometry_msgs kdl_conversions)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ros_control_boilerplate interactive_markers geometry_msgs sensor_msgs control_msgs kdl_parser tf2_ros tf2_geometry_msgs kdl_conversions)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(sim_hw_main src/sim_hw_main.cpp)
target_link_libraries(sim_hw_main ${catkin_LIBRARIES})

add_executable(interactive_marker_bridge src/interactive_marker_bridge.cpp)
target_link_libraries(interactive_marker_bridge ${catkin_LIBRARIES})

add_executable(kinematics_planner src/kinematics_planner.cpp)
target_link_libraries(kinematics_planner ${catkin_LIBRARIES})
