#!/usr/bin/env bash
cd ..

colcon build 
# --packages-select odometry_interfaces_pkg odometry_pkg

source install/setup.bash
# ros2 launch odometry_pkg test_acceleration_simulator.launch.xml
ros2 launch odometry_pkg mecanum_wheels.launch.xml