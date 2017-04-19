cmake_minimum_required(VERSION 2.8.3)
project(kiba_trajectory)

find_package(catkin REQUIRED COMPONENTS
  roscpp kiba_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp kiba_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})
