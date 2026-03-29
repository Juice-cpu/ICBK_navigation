cmake_minimum_required(VERSION 3.10)
project(patrol)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Enforcing cleaner code
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Automatically find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp_components REQUIRED) 
find_package(geometry_msgs REQUIRED)

ament_auto_find_build_dependencies()

# Create executable for the patrol node (instead of a shared library)
ament_auto_add_executable(patrol_node
  src/patrol_node.cpp
)

# Correct target for linking the serial library
target_link_libraries(patrol_node serial)

# Link dependencies to the executable
ament_target_dependencies(patrol_node
  rclcpp
  rclcpp_components
  geometry_msgs
)

# Install the executable
install(TARGETS
  patrol_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and other resources
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Automatically package everything in share/
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
