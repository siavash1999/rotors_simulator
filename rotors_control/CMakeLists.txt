cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
	control_toolbox
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES lee_position_controller lqr_controller pid_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(lee_position_controller
  src/library/lee_position_controller.cpp
)

add_library(lqr_controller
  src/library/lqr_controller.cpp
)

add_library(pid_controller
  src/library/pid_controller.cpp
)

target_link_libraries(lee_position_controller ${catkin_LIBRARIES})
add_dependencies(lee_position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(lqr_controller ${catkin_LIBRARIES})
add_dependencies(lqr_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller ${catkin_EXPORTED_TARGETS})

add_executable(lee_position_controller_node src/nodes/lee_position_controller_node.cpp)
add_dependencies(lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lee_position_controller_node
  lee_position_controller ${catkin_LIBRARIES})

add_executable(lqr_controller_node src/nodes/lqr_controller_node.cpp)
add_dependencies(lqr_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lqr_controller_node
  lqr_controller ${catkin_LIBRARIES})

  add_executable(pid_controller_node src/nodes/pid_controller_node.cpp)
  add_dependencies(pid_controller_node ${catkin_EXPORTED_TARGETS})
  target_link_libraries(pid_controller_node
    pid_controller ${catkin_LIBRARIES})

install(TARGETS lee_position_controller lqr_controller pid_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS lee_position_controller_node lqr_controller_node pid_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
