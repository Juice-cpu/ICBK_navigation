cmake_minimum_required(VERSION 3.10)
project(referee_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 查找libserial库
find_path(LIBSERIAL_INCLUDE_DIR 
  NAMES serial/serial.h
  PATHS /usr/include /usr/local/include
)

find_library(LIBSERIAL_LIBRARY 
  NAMES serial libserial
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

message(STATUS "Found libserial include: ${LIBSERIAL_INCLUDE_DIR}")
message(STATUS "Found libserial library: ${LIBSERIAL_LIBRARY}")

# 检查是否找到libserial
if(NOT LIBSERIAL_INCLUDE_DIR OR NOT LIBSERIAL_LIBRARY)
  message(FATAL_ERROR "Could not find libserial. Please install it: sudo apt-get install libserial-dev")
endif()

# 添加可执行文件
add_executable(referee_receiver_node src/referee_receiver.cpp)

# 包含目录
target_include_directories(referee_receiver_node PUBLIC
  ${LIBSERIAL_INCLUDE_DIR}
)

# 链接库
target_link_libraries(referee_receiver_node
  ${LIBSERIAL_LIBRARY}
  rclcpp::rclcpp
  std_msgs::std_msgs__rosidl_typesupport_cpp
)

install(TARGETS
  referee_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
