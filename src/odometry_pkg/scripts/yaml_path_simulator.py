#!/usr/bin/env python3
"""
YAML Path Simulator - Load and execute motion paths from YAML files

This simulator loads path configurations from YAML files and publishes
either acceleration data (for IMU) or velocity data (for mecanum wheels).

Supports three interpolation types:
- constant: Fixed value over interval
- linear: a(t) = m * t_rel + b
- parabolic: a(t) = a * t_rel² + b * t_rel + c

where t_rel = t - t_start for each segment
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from odometry_interfaces_pkg.msg import AccelerationData, PositionData
import yaml
import os
from typing import Tuple, Dict, Any, List, Optional
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory


@dataclass
class Segment:
    """Represents a single path segment"""
    start_time: float
    end_time: float
    seg_type: str  # 'constant', 'linear', 'parabolic'
    accel_x: Any  # float or dict with coefficients
    accel_y: Any
    accel_z: Any
    velocity_x: Any  # for mecanum mode
    velocity_y: Any
    omega: Any


class YAMLPathSimulator(Node):
    def __init__(self):
        super().__init__('yaml_path_simulator')

        # Declare parameters
        self.declare_parameter('path_file', '')
        self.declare_parameter('sensor_type', 'imu')  # 'imu' or 'mecanum'
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_alpha', 0.0)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_x', 0.30)
        self.declare_parameter('wheel_base_y', 0.25)
        self.declare_parameter('loop', False)  # Whether to loop the path
        self.declare_parameter('export_csv', True)  # Export to CSV file
        self.declare_parameter('csv_output_dir', 'csvexport')  # Output directory

        # Get parameters
        self.path_file = self.get_parameter('path_file').value
        self.sensor_type = self.get_parameter('sensor_type').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_alpha = self.get_parameter('initial_alpha').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value
        self.loop = self.get_parameter('loop').value
        self.export_csv = self.get_parameter('export_csv').value
        self.csv_output_dir = self.get_parameter('csv_output_dir').value

        # Validate sensor type
        if self.sensor_type not in ['imu', 'mecanum']:
            self.get_logger().error(f"Invalid sensor_type: {self.sensor_type}. Must be 'imu' or 'mecanum'")
            raise ValueError(f"Invalid sensor_type: {self.sensor_type}")

        # Load path configuration
        if not self.path_file:
            self.get_logger().error("No path_file specified!")
            raise ValueError("path_file parameter is required")

        self.path_config = self.load_yaml_path(self.path_file)
        self.segments = self.parse_segments(self.path_config)
        self.duration = self.path_config.get('duration', 30.0)
        
        # Override rate if specified in YAML
        yaml_rate = self.path_config.get('sample_rate_hz')
        if yaml_rate:
            self.rate_hz = yaml_rate

        # Calculate dt
        self.dt = 1.0 / self.rate_hz

        # State tracking
        self.sim_time = 0.0
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        
        # CSV export data
        self.csv_data = []
        self.csv_start_time = None
        self.csv_exported = False

        # Create publishers based on sensor type
        if self.sensor_type == 'imu':
            self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
            self.accel_pub = self.create_publisher(
                AccelerationData, '/simulator/acceleration', 10)
        else:  # mecanum
            self.wheel_vel_pub = self.create_publisher(
                Float64MultiArray, '/wheel_encoders/velocities', 10)
            self.joint_state_pub = self.create_publisher(
                JointState, '/joint_states', 10)

        # Reset publisher (common for both modes)
        self.reset_pub = self.create_publisher(
            PositionData, '/position/corrected', 10)

        # Timer for publishing
        self.timer = self.create_timer(self.dt, self.publish_data)

        # Send initial reset after startup
        self.initial_reset_timer = self.create_timer(0.5, self.send_initial_reset)

        self.get_logger().info(f"YAML Path Simulator started")
        self.get_logger().info(f"  Path file: {self.path_file}")
        self.get_logger().info(f"  Sensor type: {self.sensor_type}")
        self.get_logger().info(f"  Duration: {self.duration}s")
        self.get_logger().info(f"  Rate: {self.rate_hz} Hz")
        self.get_logger().info(f"  Segments: {len(self.segments)}")
        if self.export_csv:
            self.get_logger().info(f"  CSV export: enabled -> {self.csv_output_dir}/")
        self.log_segments()

    def load_yaml_path(self, file_path: str) -> Dict[str, Any]:
        """Load and parse YAML path configuration file"""
        # Handle relative paths - try package share directory
        if not os.path.isabs(file_path):
            try:
                pkg_share = get_package_share_directory('odometry_pkg')
                file_path = os.path.join(pkg_share, file_path)
            except Exception:
                pass

        if not os.path.exists(file_path):
            self.get_logger().error(f"Path file not found: {file_path}")
            raise FileNotFoundError(f"Path file not found: {file_path}")

        with open(file_path, 'r') as f:
            config = yaml.safe_load(f)

        if 'path' not in config:
            self.get_logger().error("YAML file must contain a 'path' key")
            raise ValueError("YAML file must contain a 'path' key")

        return config['path']

    def parse_segments(self, path_config: Dict[str, Any]) -> List[Segment]:
        """Parse segments from path configuration"""
        segments = []
        raw_segments = path_config.get('segments', [])

        for seg in raw_segments:
            interval = seg.get('interval', [0.0, 0.0])
            seg_type = seg.get('type', 'constant')

            segment = Segment(
                start_time=interval[0],
                end_time=interval[1],
                seg_type=seg_type,
                accel_x=seg.get('accel_x', 0.0),
                accel_y=seg.get('accel_y', 0.0),
                accel_z=seg.get('accel_z', 0.0),
                velocity_x=seg.get('velocity_x', 0.0),
                velocity_y=seg.get('velocity_y', 0.0),
                omega=seg.get('omega', 0.0)
            )
            segments.append(segment)

        # Sort by start time
        segments.sort(key=lambda s: s.start_time)
        return segments

    def log_segments(self):
        """Log all path segments"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("Path Segments:")
        for i, seg in enumerate(self.segments):
            if self.sensor_type == 'imu':
                self.get_logger().info(
                    f"  [{i}] {seg.start_time:.1f}s - {seg.end_time:.1f}s: "
                    f"{seg.seg_type} | ax={seg.accel_x}, ay={seg.accel_y}, az={seg.accel_z}")
            else:
                self.get_logger().info(
                    f"  [{i}] {seg.start_time:.1f}s - {seg.end_time:.1f}s: "
                    f"{seg.seg_type} | vx={seg.velocity_x}, vy={seg.velocity_y}, ω={seg.omega}")
        self.get_logger().info("=" * 50)

    def send_initial_reset(self):
        """Send initial position reset"""
        reset_msg = PositionData()
        reset_msg.x = self.initial_x
        reset_msg.y = self.initial_y
        reset_msg.z = 0.0
        reset_msg.alpha = self.initial_alpha
        reset_msg.initial_vx = 0.0
        reset_msg.initial_vy = 0.0
        self.reset_pub.publish(reset_msg)
        self.get_logger().info(
            f"Initial reset: ({self.initial_x}, {self.initial_y}, α={self.initial_alpha})")
        
        # Cancel one-shot timer
        self.initial_reset_timer.cancel()
        self.destroy_timer(self.initial_reset_timer)

    def get_current_segment(self, t: float) -> Optional[Segment]:
        """Get the segment that contains time t"""
        for seg in self.segments:
            if seg.start_time <= t < seg.end_time:
                return seg
        # If past all segments, use the last one
        if self.segments and t >= self.segments[-1].end_time:
            return self.segments[-1]
        return None

    def evaluate_value(self, value: Any, t_rel: float, seg_type: str) -> float:
        """
        Evaluate a value at relative time t_rel based on segment type.
        
        For constant: value is a float
        For linear: value is a dict with 'm' and 'b' -> m * t_rel + b
        For parabolic: value is a dict with 'a', 'b', 'c' -> a * t_rel² + b * t_rel + c
        """
        if value is None:
            return 0.0

        # If it's already a number, return it (constant)
        if isinstance(value, (int, float)):
            return float(value)

        # If it's a dict, evaluate based on type
        if isinstance(value, dict):
            if seg_type == 'linear':
                m = value.get('m', 0.0)
                b = value.get('b', 0.0)
                return m * t_rel + b
            elif seg_type == 'parabolic':
                a = value.get('a', 0.0)
                b = value.get('b', 0.0)
                c = value.get('c', 0.0)
                return a * t_rel * t_rel + b * t_rel + c

        return 0.0

    def get_values_at_time(self, t: float) -> Tuple[float, float, float]:
        """
        Get acceleration (for IMU) or velocity (for mecanum) at time t.
        Returns (x, y, z) for IMU or (vx, vy, omega) for mecanum.
        """
        segment = self.get_current_segment(t)
        
        if segment is None:
            return (0.0, 0.0, 0.0)

        t_rel = t - segment.start_time
        seg_type = segment.seg_type

        if self.sensor_type == 'imu':
            ax = self.evaluate_value(segment.accel_x, t_rel, seg_type)
            ay = self.evaluate_value(segment.accel_y, t_rel, seg_type)
            az = self.evaluate_value(segment.accel_z, t_rel, seg_type)
            return (ax, ay, az)
        else:  # mecanum
            vx = self.evaluate_value(segment.velocity_x, t_rel, seg_type)
            vy = self.evaluate_value(segment.velocity_y, t_rel, seg_type)
            omega = self.evaluate_value(segment.omega, t_rel, seg_type)
            return (vx, vy, omega)

    def publish_data(self):
        """Timer callback to publish data"""
        # Handle looping
        if self.loop and self.sim_time >= self.duration:
            self.sim_time = 0.0
            self.get_logger().info("Path looped - restarting from beginning")

        # Get current values
        x_val, y_val, z_val = self.get_values_at_time(self.sim_time)

        current_time = self.get_clock().now().to_msg()

        if self.sensor_type == 'imu':
            self.publish_imu_data(x_val, y_val, z_val, current_time)
        else:
            self.publish_mecanum_data(x_val, y_val, z_val, current_time)

        # Log every 2 seconds
        if int(self.sim_time * 10) % 20 == 0 and self.sim_time > 0:
            segment = self.get_current_segment(self.sim_time)
            seg_name = segment.seg_type if segment else "none"
            if self.sensor_type == 'imu':
                self.get_logger().info(
                    f"[t={self.sim_time:.1f}s] {seg_name}: "
                    f"ax={x_val:.4f}, ay={y_val:.4f}, az={z_val:.4f}")
            else:
                self.get_logger().info(
                    f"[t={self.sim_time:.1f}s] {seg_name}: "
                    f"vx={x_val:.3f}, vy={y_val:.3f}, ω={z_val:.3f}")

        self.sim_time += self.dt
        
        # Export CSV when simulation ends (for IMU mode)
        if self.export_csv and self.sensor_type == 'imu' and self.sim_time >= self.duration and not self.csv_exported:
            self.export_csv_file()
            self.csv_exported = True

    def publish_imu_data(self, ax: float, ay: float, az: float, stamp):
        """Publish IMU acceleration data"""
        # Publish standard IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        # Publish custom AccelerationData message
        accel_msg = AccelerationData()
        accel_msg.header.stamp = stamp
        accel_msg.header.frame_id = 'base_link'
        accel_msg.linear_x = ax
        accel_msg.linear_y = ay
        accel_msg.linear_z = az
        accel_msg.angular_z = 0.0
        self.accel_pub.publish(accel_msg)
        
        # Store data for CSV export
        if self.export_csv:
            if self.csv_start_time is None:
                self.csv_start_time = self.sim_time
            self.csv_data.append({
                'time': self.sim_time,
                'linear_accel_x': ax,
                'linear_accel_y': ay,
                'linear_accel_z': az,
                'angular_vel_x': 0.0,
                'angular_vel_y': 0.0,
                'angular_vel_z': 0.0
            })

    def publish_mecanum_data(self, vx: float, vy: float, omega: float, stamp):
        """Publish mecanum wheel velocities"""
        # Update tracking
        self.current_vx = vx
        self.current_vy = vy
        self.current_omega = omega

        # Calculate mecanum wheel velocities using forward kinematics
        # w1 = (1/r) * (vx - vy - k*omega)  # FL
        # w2 = (1/r) * (vx + vy + k*omega)  # FR
        # w3 = (1/r) * (vx - vy + k*omega)  # RR (note: different from some conventions)
        # w4 = (1/r) * (vx + vy - k*omega)  # RL
        lx = self.wheel_base_x / 2.0
        ly = self.wheel_base_y / 2.0
        k = lx + ly
        r = self.wheel_radius

        w1 = (vx - vy - k * omega) / r  # FL
        w2 = (vx + vy + k * omega) / r  # FR
        w3 = (vx - vy + k * omega) / r  # RR
        w4 = (vx + vy - k * omega) / r  # RL

        # Update wheel angles
        for i, w in enumerate([w1, w2, w3, w4]):
            self.wheel_angles[i] += w * self.dt

        # Publish wheel velocities
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [w1, w2, w3, w4]
        self.wheel_vel_pub.publish(wheel_msg)

        # Publish joint states for visualization
        js = JointState()
        js.header.stamp = stamp
        js.name = ["wheel_fl_joint", "wheel_fr_joint", 
                   "wheel_rr_joint", "wheel_rl_joint"]
        js.position = list(self.wheel_angles)
        js.velocity = [w1, w2, w3, w4]
        self.joint_state_pub.publish(js)

    def export_csv_file(self):
        """Export collected data to CSV file in the same format as example CSVs"""
        if not self.csv_data:
            self.get_logger().warn("No data to export")
            return
        
        # Create output directory if it doesn't exist
        # Use the workspace directory (where the launch was run from)
        output_dir = os.path.join(os.getcwd(), self.csv_output_dir)
        os.makedirs(output_dir, exist_ok=True)
        
        # Generate filename from path config name
        path_name = self.path_config.get('name', 'simulation')
        # Clean up the name for use as filename
        filename = path_name.replace(' ', '_').replace('/', '_') + '.csv'
        filepath = os.path.join(output_dir, filename)
        
        # Write CSV file
        with open(filepath, 'w') as f:
            # Write header
            f.write('time,linear_accel_x,linear_accel_y,linear_accel_z,angular_vel_x,angular_vel_y,angular_vel_z\n')
            
            # Write data rows
            for row in self.csv_data:
                f.write(f"{row['time']},{row['linear_accel_x']},{row['linear_accel_y']},"
                        f"{row['linear_accel_z']},{row['angular_vel_x']},{row['angular_vel_y']},"
                        f"{row['angular_vel_z']}\n")
        
        self.get_logger().info(f"CSV exported: {filepath}")
        self.get_logger().info(f"  Total samples: {len(self.csv_data)}")
        self.get_logger().info(f"  Duration: {self.csv_data[-1]['time']:.2f}s")

def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = YAMLPathSimulator()
        rclpy.spin(node)
    except (ValueError, FileNotFoundError) as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        # Export CSV before shutdown
        if node is not None and node.export_csv and node.csv_data and not node.csv_exported:
            node.export_csv_file()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
