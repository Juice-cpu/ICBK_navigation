cmake_minimum_required(VERSION 3.10)
project(multi_point_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(ament_cmake_auto REQUIRED)
#ament_auto_find_build_dependencies()

# Include header files
include_directories(include)

# Create executable target
add_executable(multi_point_navigation_executable src/multi_point_navigation.cpp)

# Create library target (with a different name)
# ament_auto_add_library(multi_point_navigation_lib SHARED
#   DIRECTORY src
# )

# Set include directories for both the executable and the library
target_include_directories(multi_point_navigation_executable PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(multi_point_navigation_executable PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Link dependencies to the executable
ament_target_dependencies(multi_point_navigation_executable
  rclcpp
  std_msgs
  geometry_msgs
  nav2_msgs
  nav2_util
  tf2_ros
  tf2_geometry_msgs
)

# Install executable and library
install(TARGETS multi_point_navigation_executable
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

# ament_auto_package(
#   INSTALL_TO_SHARE
#   config
#   launch
# )
