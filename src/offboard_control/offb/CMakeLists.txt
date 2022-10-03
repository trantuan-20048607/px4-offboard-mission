cmake_minimum_required(VERSION 2.8.3)
project(offb)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Optimiation options
add_compile_options(-Ofast -flto -march=native)

## Show all warnings
add_compile_options(-Wall)

find_package(catkin REQUIRED
  COMPONENTS
  message_generation  
  roscpp
  move_base_msgs
  geometry_msgs
  sensor_msgs
  mavros
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  mavros_msgs
  actionlib
  trajectory_msgs
  pluginlib
  offb_msgs
  serial
  cv_bridge
  image_geometry
)

find_package(cmake_modules REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES vision_to_mavros
  # CATKIN_DEPENDS roscpp rospy tf
  # DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(monitor_node src/monitor.cpp)
add_dependencies(monitor_node ${catkin_EXPORTED_TARGETS} offb_msgs_gencpp)
target_link_libraries(monitor_node ${catkin_LIBRARIES})

add_executable(set_mode_node src/set_mode.cpp)
target_link_libraries(set_mode_node ${catkin_LIBRARIES})

add_executable(send_info_node src/send_info.cpp)
target_link_libraries(send_info_node ${catkin_LIBRARIES})

add_executable(com_node src/com.cpp)
target_link_libraries(com_node ${catkin_LIBRARIES})

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(servo_control_node src/servo_control.cpp)
target_link_libraries(servo_control_node ${catkin_LIBRARIES})

add_executable(vision_node src/vision.cpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(
  TARGETS  # offb_node monitor_node set_mode_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
