cmake_minimum_required(VERSION 2.8.3)
project(my_first_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(FILES SetFloat64.srv)
generate_messages()
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_first_node src/my_first_node.cpp)
target_link_libraries(my_first_node ${catkin_LIBRARIES})
add_dependencies(my_first_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})
