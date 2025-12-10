#!/usr/bin/env bash
cd ..

colcon build 

source install/setup.bash

ros2 launch odometry_pkg esp32_imu_position.launch.xml