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

add_executable(station_monitor_node src/station/monitor.cpp)
add_dependencies(station_monitor_node ${catkin_EXPORTED_TARGETS} offb_msgs_gencpp)
target_link_libraries(station_monitor_node ${catkin_LIBRARIES})

add_executable(station_mode_control_node src/station/mode_control.cpp)
target_link_libraries(station_mode_control_node ${catkin_LIBRARIES})

add_executable(station_fly_control_node src/station/fly_control.cpp)
target_link_libraries(station_fly_control_node ${catkin_LIBRARIES})

add_executable(station_servo_control_node src/station/servo_control.cpp)
target_link_libraries(station_servo_control_node ${catkin_LIBRARIES})

add_executable(station_watchdog_node src/station/watchdog.cpp)
target_link_libraries(station_watchdog_node ${catkin_LIBRARIES})

add_executable(uav_servo_node src/uav/servo.cpp)
target_link_libraries(uav_servo_node ${catkin_LIBRARIES})

add_executable(uav_vision_node src/uav/vision.cpp)
target_link_libraries(uav_vision_node ${catkin_LIBRARIES})

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Mark executables and/or libraries for installation
install(
  TARGETS  # offb_node state_monitor_node mode_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
