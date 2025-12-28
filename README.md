# ROS2_Odometry

ROS 2 workspace for experimenting with odometry estimation and visualization on a small mecanum robot. It includes:

- A mecanum URDF with per-wheel TF frames and STL meshes for RViz rendering.
- Simulated wheel encoders and a mecanum kinematics approximator that publishes pose and path.
- A position visualizer node that broadcasts TF, publishes nav_msgs/Path, and shows the robot model in RViz.
- Launch files for a simulated wheel workflow and an ESP32 IMU workflow, both wiring in robot_state_publisher, the visualizer, and an optional RViz session.
- A SQLite database handler for logging wheel/IMU velocity and position topics during runs.

Use the Python or XML launch variants to start either the mecanum wheel simulation or the ESP32 IMU pipeline, then inspect TF, path, and mesh overlays in RViz.

## IMU Simulator - Custom Motion Paths

The IMU simulator supports configurable motion paths via launch file parameters. You can define complex trajectories with precise control over velocities, rotations, and timing.

### Quick Start

Launch the IMU simulator with RViz visualization:

```bash
colcon build --packages-select odometry_pkg
source install/setup.zsh
ros2 launch odometry_pkg imu_simulator.launch.xml
```
or for bash:
```bash
colcon build --packages-select odometry_pkg
source install/setup.bash
ros2 launch odometry_pkg imu_simulator.launch.xml
```

### Path Parameters

Define custom motion paths using these parameter arrays:

- **`path_times`**: Start times for each phase (in seconds). Must be monotonically increasing.
- **`path_velocities_x`**: Forward velocity (m/s) for each phase. Positive = forward, negative = backward.
- **`path_velocities_y`**: Lateral velocity (m/s) for each phase. Positive = left, negative = right.
- **`path_rotations`**: Angular velocity (rad/s) for each phase. Positive = counter-clockwise.

**Important**: All arrays must have the same length. The simulator calculates required accelerations to transition between phases.

### Example: Complex Motion Sequence

This example demonstrates a 7-phase motion path (45 seconds total):

```xml
<!-- Phase 1 (0-5s):   Stand still (0 m/s) -->
<!-- Phase 2 (5-15s):  Drive 10s at 0.25 m/s -->
<!-- Phase 3 (15-20s): Drive 5s at 0.5 m/s -->
<!-- Phase 4 (20-30s): Drive 10s at 0.25 m/s -->
<!-- Phase 5 (30-35s): Stand still 5s (0 m/s) -->
<!-- Phase 6 (35-40s): Turn 90° (0.785 rad/s) -->
<!-- Phase 7 (40s+):   Stand still -->
<param name="path_times" value="[0.0, 5.0, 15.0, 20.0, 30.0, 35.0, 40.0, 45.0]" />
<param name="path_velocities_x" value="[0.0, 0.25, 0.5, 0.25, 0.0, 0.0, 0.0, 0.0]" />
<param name="path_velocities_y" value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" />
<param name="path_rotations" value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.785, 0.0, 0.0]" />
```

**Key points:**

- Times are **start times**, not durations (e.g., phase 2 starts at t=5s, not after 5s)
- Set `cycle_time` to match your last time value (45.0 in this example)
- Ensure `initial_velocity` matches `path_velocities_x[0]` to prevent premature motion

### Command Line Custom Paths

You can pass custom path arrays directly via the command line:

```bash
# Square path (mecanum strafe motion)
ros2 launch odometry_pkg imu_simulator.launch.xml \
  path_times:="[0.0, 3.0, 6.0, 9.0, 12.0]" \
  path_velocities_x:="[0.3, 0.0, -0.3, 0.0, 0.0]" \
  path_velocities_y:="[0.0, 0.3, 0.0, -0.3, 0.0]" \
  path_rotations:="[0.0, 0.0, 0.0, 0.0, 0.0]"

# Forward, strafe, rotate sequence
ros2 launch odometry_pkg imu_simulator.launch.xml \
  path_times:="[0.0, 2.0, 4.0, 6.0, 8.0]" \
  path_velocities_x:="[0.3, 0.0, -0.3, 0.0, 0.0]" \
  path_velocities_y:="[0.0, 0.3, 0.0, -0.3, 0.0]" \
  path_rotations:="[0.0, 0.0, 0.0, 0.0, 0.5]"
```

### Editing Default Path in Launch File

Alternatively, edit the defaults directly in `imu_simulator.launch.xml`:

```xml
<!-- Custom path parameters (overrides simple mode if provided) -->
<arg name="path_times" default="[0.0, 3.0, 6.0, 9.0, 12.0]" />
<arg name="path_velocities_x" default="[0.3, 0.0, -0.3, 0.0, 0.0]" />
<arg name="path_velocities_y" default="[0.0, 0.3, 0.0, -0.3, 0.0]" />
<arg name="path_rotations" default="[0.0, 0.0, 0.0, 0.0, 0.0]" />
```

Then simply run:

```bash
ros2 launch odometry_pkg imu_simulator.launch.xml
```

### Predefined Patterns

Use the alternative launch file with predefined motion patterns:

```bash
ros2 launch odometry_pkg imu_custom_path.launch.xml motion_pattern:=circular
ros2 launch odometry_pkg imu_custom_path.launch.xml motion_pattern:=square
ros2 launch odometry_pkg imu_custom_path.launch.xml motion_pattern:=stop_and_go
```

Available patterns: `stop_and_go`, `circular`, `square`, `strafe`, `complex`

### Other Parameters

- **`initial_velocity`** (default: 0.0): Initial robot velocity (m/s)
- **`initial_position_x/y`** (default: 0.0): Starting position (meters)
- **`initial_alpha`** (default: 0.0): Starting orientation (radians)
- **`cycle_time`** (default: 45.0): Duration of one complete path cycle (seconds)
- **`publish_rate_hz`** (default: 50): IMU data publishing frequency
- **`noise_std`** (default: 0.001): Acceleration noise standard deviation (m/s²)
- **`wheel_radius`** (default: 0.05): Mecanum wheel radius (meters)
- **`wheel_base_x`** (default: 0.30): Front-to-rear wheel distance (meters)
- **`wheel_base_y`** (default: 0.25): Left-to-right wheel distance (meters)

### Runtime Parameter Updates

Parameters can be changed at runtime:

```bash
ros2 param set /imu_simulator path_velocities_x "[0.0, 0.5, 1.0]"
ros2 param set /imu_simulator path_times "[0.0, 10.0, 20.0]"
```

The simulator automatically recalculates acceleration intervals when path parameters change.

### Tips

1. **Smooth transitions**: The simulator handles accelerations automatically between phases
2. **Rotation in place**: Set velocities to 0.0 and only use `path_rotations`
3. **Strafe motion**: Use `path_velocities_y` for lateral movement (mecanum wheels)
4. **Complex paths**: Combine forward, lateral, and rotational motion simultaneously
5. **Database logging**: All IMU and position data is automatically logged to SQLite

For more examples, see `config/imu_path_examples.yaml`.
