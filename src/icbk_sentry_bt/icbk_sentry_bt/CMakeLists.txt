cmake_minimum_required(VERSION 3.10)
project(icbk_sentry_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(
  behavior_tree_main
  src/behavior_tree_main.cpp
  src/patrol_between_points.cpp
  src/referee_receiver.cpp
)

target_include_directories(behavior_tree_main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(behavior_tree_main
  "rclcpp"
  "ament_index_cpp"
  "behaviortree_cpp_v3"
  "std_msgs"
  "geometry_msgs"
  "nav2_msgs"
  "rm_interfaces"
  "rclcpp_action"
  "tf2_ros"
  "tf2_geometry_msgs")

find_library(LIBSERIAL_LIBRARY serial)
if(NOT LIBSERIAL_LIBRARY)
  set(LIBSERIAL_LIBRARY /usr/lib/x86_64-linux-gnu/libserial.so)
endif()
target_link_libraries(behavior_tree_main ${LIBSERIAL_LIBRARY})

target_link_libraries(behavior_tree_main /usr/lib/x86_64-linux-gnu/libserial.so)

install(TARGETS
  behavior_tree_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES
  behavior_trees/sentry_behavior.xml
  behavior_trees/rmul_2026.xml
  DESTINATION share/${PROJECT_NAME}/behavior_trees/
)


ament_package()