cmake_minimum_required(VERSION 3.8)
project(serpentine_patrol)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Define the executable target
add_executable(serpentine_patrol_node src/serpentine_patrol_node.cpp)
add_executable(serpentine_patrol_through_poses_node src/serpentine_patrol_through_poses_node.cpp)
add_executable(serpentine_patrol_open_loop_node src/serpentine_patrol_open_loop_node.cpp)

# Link libraries
ament_target_dependencies(serpentine_patrol_node
  rclcpp
  geometry_msgs
  std_msgs
  nav2_msgs
  rclcpp_action
)

ament_target_dependencies(serpentine_patrol_through_poses_node
  rclcpp
  geometry_msgs
  std_msgs
  nav2_msgs
  rclcpp_action
)

ament_target_dependencies(serpentine_patrol_open_loop_node
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# Install the executable
install(TARGETS
  serpentine_patrol_node
  serpentine_patrol_through_poses_node
  serpentine_patrol_open_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Package setup
ament_package()
