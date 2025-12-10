# Custom Position message for odometry
# Position in fixed frame (map)

std_msgs/Header header

# Position in meters
float64 x
float64 y
float64 z

# Angular orientation (rotation around z-axis, "north" of the map) in radians
float64 alpha

# Initial velocities (used when resetting the integrator)
float64 initial_vx
float64 initial_vy
