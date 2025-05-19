cmake_minimum_required(VERSION 3.8)
project(orbbec_rtsp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orbbec_camera REQUIRED)  # 修改此处

# 查找GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# 添加可执行文件
add_executable(orbbec_rtsp_server src/orbbec_rtsp_server.cpp)
ament_target_dependencies(
  orbbec_rtsp_server
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  orbbec_camera  # 添加此处
)

# 链接GStreamer库
target_include_directories(orbbec_rtsp_server PUBLIC
  ${GSTREAMER_INCLUDE_DIRS}
)
target_link_libraries(orbbec_rtsp_server
  ${GSTREAMER_LIBRARIES}
)

# 安装可执行文件
install(TARGETS
  orbbec_rtsp_server
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件和启动文件
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()