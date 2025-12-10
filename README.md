# ROS2_Odometry

ROS 2 workspace for experimenting with odometry estimation and visualization on a small mecanum robot. It includes:

- A mecanum URDF with per-wheel TF frames and STL meshes for RViz rendering.
- Simulated wheel encoders and a mecanum kinematics approximator that publishes pose and path.
- A position visualizer node that broadcasts TF, publishes nav_msgs/Path, and shows the robot model in RViz.
- Launch files for a simulated wheel workflow and an ESP32 IMU workflow, both wiring in robot_state_publisher, the visualizer, and an optional RViz session.
- A SQLite database handler for logging wheel/IMU velocity and position topics during runs.

Use the XML launch variants to start either the mecanum wheel simulation or the ESP32 IMU pipeline, then inspect TF, path, and mesh overlays in RViz.

## Running

Go to the `scripts` folder

### Run using real IMU Sensor data

```bash
./run.sh
```

### Run in a simulation

```bash
run_simulation.sh
```
