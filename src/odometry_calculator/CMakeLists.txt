cmake_minimum_required(VERSION 3.8)
project(odometry_calculator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 添加可执行文件
add_executable(odometer_node src/odometry_calculator.cpp)
ament_target_dependencies(odometer_node rclcpp nav_msgs std_msgs)

# 安装可执行文件
install(TARGETS
  odometer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()    