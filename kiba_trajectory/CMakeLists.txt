cmake_minimum_required(VERSION 2.8.3)
project(kiba_trajectory)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp kiba_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp kiba_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

catkin_add_gtest(${PROJECT_NAME}-test 
  test/main.cpp 
  test/${PROJECT_NAME}/spline.cpp
  test/${PROJECT_NAME}/conversions.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${catkin_LIBRARIES})
endif()
