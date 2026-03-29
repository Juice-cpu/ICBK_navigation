cmake_minimum_required(VERSION 3.10)
project(armor_chaser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(chase_state_machine_node src/chase_state_machine.cpp)
ament_target_dependencies(chase_state_machine_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  rm_interfaces
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(TARGETS
  chase_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
