cmake_minimum_required(VERSION 3.10)
project(real_vel_transform)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

ament_auto_find_build_dependencies()

# Add the library for the package
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/real_vel_transform.cpp
)

# Ensure headers are visible during build/install.
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link the serial library
target_link_libraries(${PROJECT_NAME} serial)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  nav_msgs
  tf2_ros
  std_msgs
  rm_interfaces
)

# Register the node component with ROS 2
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN real_vel_transform::RealVelTransform
  EXECUTABLE ${PROJECT_NAME}_node
)

# Linting (if testing enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files and other resources
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install the node executable
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Automatically package everything in share/
ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
