cmake_minimum_required(VERSION 2.8.3)
project(kiba_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp ros_control_boilerplate)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp ros_control_boilerplate)

include_directories(
  include
  ${catkin_INCLUDE_DIRS})

add_executable(sim_hw_main src/sim_hw_main.cpp)
target_link_libraries(sim_hw_main ${catkin_LIBRARIES})
