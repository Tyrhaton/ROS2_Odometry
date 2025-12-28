#!/usr/bin/env bash
cd ..

colcon build 

source install/setup.bash

# ros2 launch odometry_pkg mecanum_wheels.launch.xml
ros2 launch odometry_pkg mecanum_wheels.launch.xml \
  motion_mode:=accel_profile \
  accel_profile_path:=/home/tyrhaton/Projects/MinorEmbeddedSystems/ROS2_Odometry/src/odometry_pkg/config/accel_profile.yaml
