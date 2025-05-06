cmake_minimum_required(VERSION 3.8)
project(random_odom_pkg)

# 设置编译类型为 Debug，以便使用 GDB 调试
set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(random_odom_publisher src/random_odom_publisher.cpp)
ament_target_dependencies(random_odom_publisher rclcpp nav_msgs)

install(TARGETS
  random_odom_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()    