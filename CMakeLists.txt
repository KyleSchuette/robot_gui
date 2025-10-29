cmake_minimum_required(VERSION 3.0.2)
project(robot_gui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Catkin and ROS deps
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotinfo_msgs
  roscpp
  std_srvs
)

## OpenCV for GUI rendering
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
#  CATKIN_DEPENDS geometry_msgs nav_msgs robotinfo_msgs roscpp std_srvs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Node
add_executable(${PROJECT_NAME}_node src/robot_gui_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
