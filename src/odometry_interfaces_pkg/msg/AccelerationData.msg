# Custom Acceleration message for odometry
# Note: For actual sensor data, use sensor_msgs/Imu
# This message is for the simulator and approximator nodes

std_msgs/Header header

# Linear acceleration in m/s^2
float64 linear_x
float64 linear_y
float64 linear_z

# Angular acceleration around z-axis in rad/s^2
float64 angular_z
