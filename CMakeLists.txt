cmake_minimum_required(VERSION 3.8)
project(gps_nav_robot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs geographic_msgs
  visualization_msgs robot_localization tf2 tf2_ros tf2_geometry_msgs)

add_executable(sensor_monitor src/sensor_monitor.cpp)
ament_target_dependencies(sensor_monitor rclcpp sensor_msgs std_msgs diagnostic_msgs)

install(TARGETS gps_waypoint_follower sensor_monitor DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/xy_to_gps.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config urdf worlds rviz bt DESTINATION share/${PROJECT_NAME})

ament_package()
