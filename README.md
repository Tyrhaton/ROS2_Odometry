# ROS2_Odometry

ROS 2 workspace for experimenting with odometry estimation and visualization on a small mecanum robot. It includes:

- A mecanum URDF with per-wheel TF frames and STL meshes for RViz rendering.
- Simulated wheel encoders and a mecanum kinematics approximator that publishes pose and path.
- A position visualizer node that broadcasts TF, publishes nav_msgs/Path, and shows the robot model in RViz.
- Launch files for a simulated wheel workflow and an ESP32 IMU workflow, both wiring in robot_state_publisher, the visualizer, and an optional RViz session.
- A SQLite database handler for logging wheel/IMU velocity and position topics during runs.

Use the Python or XML launch variants to start either the mecanum wheel simulation or the ESP32 IMU pipeline, then inspect TF, path, and mesh overlays in RViz.

---

## Quick Reference - Copy & Paste Commands

### Build & Source (run once)
```bash
cd ~/school_shit_ros2/fix_imu_tilmann_assignment/ROS2_Odometry
colcon build --packages-select odometry_pkg
source install/setup.zsh   # or setup.bash
```

### IMU Simulator (Acceleration-based)

```bash
# Constant acceleration - smooth (10ms, 6 decimals)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_constant_accel.yaml \
  interval:=10 decimals:=6

# Constant acceleration - stair-stepping (100ms, 2 decimals)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_constant_accel.yaml \
  interval:=100 decimals:=2

# Linear acceleration - smooth
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=10 decimals:=6

# Linear acceleration - stair-stepping
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=100 decimals:=2

# Quadratic acceleration - smooth
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_quadratic_accel.yaml \
  interval:=10 decimals:=6

# Quadratic acceleration - stair-stepping
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_quadratic_accel.yaml \
  interval:=100 decimals:=2
```

### Mecanum Simulator (Velocity-based)

```bash
# Mecanum velocity - smooth (10ms, 6 decimals)
ros2 launch odometry_pkg yaml_path_mecanum.launch.xml \
  path_file:=config/paths/mecanum_velocity_path.yaml \
  interval:=10 decimals:=6

# Mecanum velocity - stair-stepping (100ms, 2 decimals)
ros2 launch odometry_pkg yaml_path_mecanum.launch.xml \
  path_file:=config/paths/mecanum_velocity_path.yaml \
  interval:=100 decimals:=2
```

### Parameter Explanation

| Parameter | Values | Effect |
|-----------|---------|--------|
| `interval` | `10` | 10ms = 100Hz (smooth) |
| `interval` | `100` | 100ms = 10Hz (stair-stepping visible) |
| `interval` | `500` | 500ms = 2Hz (coarse steps) |
| `decimals` | `6` | Full precision (smooth line) |
| `decimals` | `2` | Rounded to 0.01 (stair-stepping) |

---

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

---

## YAML Path Simulator

The YAML Path Simulator allows you to define custom motion paths in YAML files with support for three interpolation types:

- **Constant**: Fixed value over an interval
- **Linear**: `a(t) = m * t_rel + b` (linear ramp)
- **Quadratic**: `a(t) = a * t_rel² + b * t_rel + c` (quadratic curve)

where `t_rel = t - t_start` is the time relative to the segment start.

### Lagrange Interpolation

The simulator uses **piecewise linear Lagrange interpolation** to calculate acceleration values between defined points. This provides:

- **Smooth transitions** at any sample rate
- **Mathematically accurate** interpolation using the Lagrange polynomial formula
- **Configurable precision** via the `decimals` parameter

The Lagrange formula used for linear interpolation between two points:

$$P(t) = f(x_i) \cdot \frac{t - x_{i+1}}{x_i - x_{i+1}} + f(x_{i+1}) \cdot \frac{t - x_i}{x_{i+1} - x_i}$$

### Quick Start

```bash
# Build and source
colcon build --packages-select odometry_pkg
source install/setup.zsh

# Run with constant acceleration (IMU mode)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_constant_accel.yaml

# Run with linear acceleration - smooth (10ms interval, 6 decimals)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=10 decimals:=6

# Run with linear acceleration - "stair-stepping" effect (100ms interval, 2 decimals)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=100 decimals:=2

# Run with quadratic acceleration
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_quadratic_accel.yaml

# Run with mecanum wheel velocities
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/mecanum_velocity_path.yaml \
  sensor_type:=mecanum
```

### Sample Rate & Precision Control

The simulator provides two key parameters to control output resolution:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `interval` | `-1` | Sample interval in milliseconds. Overrides YAML `sample_rate_hz`. Examples: `10` (100Hz), `100` (10Hz), `500` (2Hz) |
| `decimals` | `6` | Decimal precision for acceleration values. Use `2` for visible "stair-stepping" (stair-stepping), `6` for smooth lines |

**Examples:**

```bash
# Smooth line (high precision, high sample rate)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=10 decimals:=6

# Visible stair-steps (low precision, low sample rate)  
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=100 decimals:=2

# Very coarse sampling (500ms = 2Hz)
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/imu_linear_accel.yaml \
  interval:=500 decimals:=2
```

**How `decimals` affects output:**

| decimals | Rounding | Effect |
|-----------|----------|--------|
| `2` | 0.01 | Values repeat → visible "stair-stepping" |
| `3` | 0.001 | Slight stepping |
| `6` | 0.000001 | Smooth line (default) |

### YAML Path File Format

#### IMU (Acceleration-based)

```yaml
path:
  name: "my_imu_path"
  duration: 30.0
  sample_rate_hz: 10  # 100ms intervals

  segments:
    # Constant acceleration
    - interval: [0.0, 5.0]
      type: constant
      accel_x: 0.0
      accel_y: 0.0
      accel_z: 0.0

    # Linear acceleration: a(t) = m * t_rel + b
    - interval: [5.0, 10.0]
      type: linear
      accel_x:
        m: 0.05   # slope
        b: 0.0    # initial value at segment start
      accel_y: 0.0
      accel_z: 0.0

    # Quadratic acceleration: a(t) = a * t_rel² + b * t_rel + c
    - interval: [10.0, 15.0]
      type: quadratic
      accel_x:
        a: -0.025  # quadratic coefficient
        b: 0.1     # linear coefficient
        c: 0.0     # constant offset
      accel_y: 0.0
      accel_z: 0.0
```

#### Mecanum Wheels (Velocity-based)

```yaml
path:
  name: "my_mecanum_path"
  duration: 30.0
  sample_rate_hz: 50

  segments:
    # Constant velocity
    - interval: [0.0, 5.0]
      type: constant
      velocity_x: 0.3  # forward (m/s)
      velocity_y: 0.0  # lateral (m/s)
      omega: 0.0       # rotation (rad/s)

    # Linear velocity ramp
    - interval: [5.0, 10.0]
      type: linear
      velocity_x:
        m: 0.1
        b: 0.3
      velocity_y: 0.0
      omega: 0.0
```

### Available Path Files

| File | Description | Sensor Type |
|------|-------------|-------------|
| `imu_constant_accel.yaml` | Constant acceleration (Assignment File 1) | IMU |
| `imu_linear_accel.yaml` | Linear acceleration (Assignment File 2) | IMU |
| `imu_quadratic_accel.yaml` | Quadratic acceleration (Assignment File 3) | IMU |
| `mecanum_velocity_path.yaml` | Example mecanum velocity profile | Mecanum |

### Launch File Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `path_file` | `config/paths/imu_constant_accel.yaml` | Path to YAML file |
| `sensor_type` | `imu` | `imu` or `mecanum` |
| `interval` | `-1` | Sample interval in ms (-1 = use YAML sample_rate_hz) |
| `decimals` | `6` | Decimal precision (2 = stair-stepping, 6 = smooth) |
| `use_rviz` | `true` | Launch RViz visualization |
| `loop` | `false` | Loop the path continuously |
| `initial_x` | `0.0` | Starting X position (m) |
| `initial_y` | `0.0` | Starting Y position (m) |

### Creating Custom Paths

1. Create a new YAML file in `config/paths/`
2. Define segments with intervals and interpolation types
3. Launch with your custom path file:

```bash
ros2 launch odometry_pkg yaml_path_simulator.launch.xml \
  path_file:=config/paths/my_custom_path.yaml \
  sensor_type:=imu
```

---

## Mecanum Velocity Path Configuration

The mecanum simulator supports multiple velocity input formats, giving you flexibility to define constant speeds, linear ramps, quadratic profiles, or direct wheel control.

### Velocity Input Formats

Each segment in a mecanum path file uses an `interval: [start, end]` to define the time range. Within that interval, you can specify velocities in four different ways:

#### 1. Constant Velocity (Scalar)

Simple constant velocity throughout the interval:

```yaml
- interval: [0.0, 5.0]
  velocity_x: 0.3    # Constant 0.3 m/s forward
  velocity_y: 0.0    # No lateral movement
  omega: 0.0         # No rotation
```

#### 2. Linear Velocity (Ramp)

Velocity changes linearly over time: `v(t) = m * t_rel + b`

Where `t_rel = t - t_start` (time relative to segment start).

```yaml
- interval: [2.0, 5.0]
  velocity_x:
    m: 0.1   # Slope: acceleration of 0.1 m/s per second
    b: 0.0   # Starting velocity at segment start
  velocity_y: 0.0
  omega: 0.0
```

**Example calculation:** At `t = 4.0` (so `t_rel = 2.0`):
`velocity_x = 0.1 * 2.0 + 0.0 = 0.2 m/s`

#### 3. Quadratic Velocity (Curve)

Velocity follows a quadratic profile: `v(t) = a * t_rel² + b * t_rel + c`

```yaml
- interval: [20.0, 25.0]
  velocity_x:
    a: -0.01    # Quadratic coefficient (negative = deceleration curve)
    b: -0.02    # Linear coefficient
    c: 0.3      # Starting velocity
  velocity_y: 0.0
  omega: 0.0
```

**Use cases:**
- Smooth acceleration/deceleration profiles
- S-curve motion approximations
- Non-linear speed ramps

#### 4. Direct Wheel Velocities

Control each wheel individually (bypasses kinematics calculation):

```yaml
- interval: [10.0, 15.0]
  omega: 0.0           # Angular velocity (can be combined)
  velocity_fl: 3.0     # Front Left wheel
  velocity_fr: -3.0    # Front Right wheel
  velocity_rl: 3.0     # Rear Left wheel
  velocity_rr: -3.0    # Rear Right wheel
```

**Wheel velocity signs:**
- Positive = wheel rotates forward
- Negative = wheel rotates backward

**Common patterns:**
| Motion | FL | FR | RL | RR |
|--------|-----|-----|-----|-----|
| Forward | + | + | + | + |
| Backward | - | - | - | - |
| Strafe Right | + | - | - | + |
| Strafe Left | - | + | + | - |
| Rotate CW | + | - | + | - |
| Rotate CCW | - | + | - | + |

### Complete Example Path

```yaml
path:
  name: "demo_path"
  duration: 30.0
  sample_rate_hz: 50

  segments:
    # Stand still (constant)
    - interval: [0.0, 2.0]
      velocity_x: 0.0
      velocity_y: 0.0
      omega: 0.0

    # Accelerate forward (linear ramp)
    - interval: [2.0, 5.0]
      velocity_x:
        m: 0.1
        b: 0.0
      velocity_y: 0.0
      omega: 0.0

    # Cruise at constant speed
    - interval: [5.0, 10.0]
      velocity_x: 0.3
      velocity_y: 0.0
      omega: 0.0

    # Direct wheel control (strafe)
    - interval: [10.0, 15.0]
      omega: 0.0
      velocity_fl: 3.0
      velocity_fr: -3.0
      velocity_rl: 3.0
      velocity_rr: -3.0

    # Circular motion (constant + rotation)
    - interval: [15.0, 20.0]
      velocity_x: 0.3
      velocity_y: 0.0
      omega: 0.4

    # Smooth deceleration (quadratic)
    - interval: [20.0, 25.0]
      velocity_x:
        a: -0.01
        b: -0.02
        c: 0.3
      velocity_y: 0.0
      omega: 0.0

    # Stop
    - interval: [25.0, 30.0]
      velocity_x: 0.0
      velocity_y: 0.0
      omega: 0.0
```

### Velocity Parameters Reference

| Parameter | Type | Description |
|-----------|------|-------------|
| `velocity_x` | scalar/linear/quadratic | Forward velocity (m/s). Positive = forward |
| `velocity_y` | scalar/linear/quadratic | Lateral velocity (m/s). Positive = left |
| `omega` | scalar | Angular velocity (rad/s). Positive = counter-clockwise |
| `velocity_fl` | scalar | Front Left wheel velocity (rad/s) |
| `velocity_fr` | scalar | Front Right wheel velocity (rad/s) |
| `velocity_rl` | scalar | Rear Left wheel velocity (rad/s) |
| `velocity_rr` | scalar | Rear Right wheel velocity (rad/s) |

### Launch Commands

```bash
# Standard mecanum simulation
ros2 launch odometry_pkg yaml_path_mecanum.launch.xml \
  path_file:=config/paths/mecanum_velocity_path.yaml

# With custom interval and precision
ros2 launch odometry_pkg yaml_path_mecanum.launch.xml \
  path_file:=config/paths/mecanum_velocity_path.yaml \
  interval:=10 decimals:=6
```
