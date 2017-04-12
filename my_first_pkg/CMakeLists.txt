cmake_minimum_required(VERSION 2.8.3)
project(my_first_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_first_node src/my_first_node.cpp)
target_link_libraries(my_first_node ${catkin_LIBRARIES})
