cmake_minimum_required(VERSION 2.8.3)
project(my_adder)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(my_adder src/my_adder.cpp)
target_link_libraries(my_adder ${catkin_LIBRARIES})
add_dependencies(my_adder 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(${PROJECT_NAME}-test 
  test/main.cpp 
  test/${PROJECT_NAME}/my_adder.cpp)

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${catkin_LIBRARIES})
endif()
