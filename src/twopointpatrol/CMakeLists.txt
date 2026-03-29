cmake_minimum_required(VERSION 3.10)
project(twopointpatrol)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # <-- 1. 添加这一行
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Define the executable target
add_executable(patroldriver src/patroldriver.cpp)

# 这行现在可以移除了，因为 find_package 和 ament_target_dependencies 会处理好
target_include_directories(patroldriver PRIVATE /opt/ros/humble/include)

# Link libraries
ament_target_dependencies(patroldriver
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_msgs  # <-- 2. 添加这一行
  tf2
  tf2_geometry_msgs
  rclcpp_action
  std_msgs
)

# Install the executable
install(TARGETS patroldriver
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# 这里也保持一致，把所有用到的依赖都导出
ament_export_dependencies(rclcpp geometry_msgs nav_msgs nav2_msgs tf2 tf2_geometry_msgs rclcpp_action std_msgs)

# Package setup
ament_package()
