# ros2_ws

## odom接收、保存、发布
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --license Apache-2.0 odometry_calculator

cd ~/ros2_ws
colcon build --packages-select odometry_calculator

source install/setup.bash

ros2 run odometry_calculator odometer_node

## 命令发布odom:
ros2 topic pub -r 1 /rtk_gps_node/odom nav_msgs/msg/Odometry '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "odom"}, child_frame_id: "base_link", pose: {pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}, twist: {twist: {linear: {x: 0.1, y: 0.2, z: 0.3}, angular: {x: 0.0, y: 0.0, z: 0.0}}, covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}}'

## 随机发布odom:
colcon build --packages-select random_odom_pkg
source ~/ros2_ws/install/setup.bash
ros2 run random_odom_pkg random_odom_publisher