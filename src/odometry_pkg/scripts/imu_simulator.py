#!/usr/bin/env python3
"""
IMU Simulator for testing odometry with configurable acceleration profile

This simulator generates IMU data following a customizable motion pattern
using acceleration intervals (constant, linear, or quadratic profiles).

Supports:
- Configurable acceleration intervals with polynomial profiles
- Runtime parameter changes (initial velocity, intervals)
- Position reset with initial velocity
- Noise simulation for realistic IMU data

Acceleration polynomial types:
- CONSTANT: a(t) = c
- LINEAR: a(t) = a*t + b  
- QUADRATIC: a(t) = a*t² + b*t + c
(time t is relative to interval start)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from odometry_interfaces_pkg.msg import AccelerationData, PositionData
import math
import random


class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')

        # Declare parameters
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('noise_std', 0.001)  # m/s² standard deviation
        self.declare_parameter('initial_velocity', 0.5)  # m/s
        self.declare_parameter('initial_position_x', 0.0)  # m
        self.declare_parameter('initial_position_y', 0.0)  # m
        self.declare_parameter('initial_alpha', 0.0)  # radians
        self.declare_parameter('cycle_time', 90.0)  # seconds
        
        # Motion path parameters - define time intervals for movement phases
        self.declare_parameter('path_times', [0.0, 5.0, 6.0, 11.0, 12.0, 90.0])
        self.declare_parameter('path_velocities_x', [0.5, 0.5, 0.0, 0.0, 0.5, 0.5])  # m/s forward
        self.declare_parameter('path_velocities_y', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # m/s lateral (strafe)
        self.declare_parameter('path_rotations', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # rad/s angular velocity
        
        # Robot geometry for joint states (matching mecanum_wheels)
        self.declare_parameter('wheel_radius', 0.05)  # meters
        self.declare_parameter('wheel_base_x', 0.30)  # meters
        self.declare_parameter('wheel_base_y', 0.25)  # meters

        # Get parameters
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.noise_std = self.get_parameter('noise_std').value
        self.initial_velocity = self.get_parameter('initial_velocity').value
        self.initial_position_x = self.get_parameter('initial_position_x').value
        self.initial_position_y = self.get_parameter('initial_position_y').value
        self.initial_alpha = self.get_parameter('initial_alpha').value
        self.cycle_time = self.get_parameter('cycle_time').value
        
        # Get motion path parameters
        self.path_times = self.get_parameter('path_times').value
        self.path_velocities_x = self.get_parameter('path_velocities_x').value
        self.path_velocities_y = self.get_parameter('path_velocities_y').value
        self.path_rotations = self.get_parameter('path_rotations').value
        
        # Robot geometry
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value

        # Build acceleration intervals from path parameters
        # This will calculate required accelerations to reach target velocities
        self.acceleration_intervals = self.build_acceleration_intervals()
        
        # Track current velocities
        self.current_velocity_x = self.initial_velocity  # Track x velocity
        self.current_velocity_y = 0.0  # Track y velocity (strafe)
        self.current_omega = 0.0  # Track angular velocity
        
        # Track wheel angles for visualization
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RR, RL

        # Publishers
        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        self.accel_pub = self.create_publisher(
            AccelerationData,
            '/simulator/acceleration',
            10
        )

        # Publisher for position reset (to set initial velocity)
        self.reset_pub = self.create_publisher(
            PositionData,
            '/position/corrected',
            10
        )
        
        # Joint state publisher for wheel visualization in RViz
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Timer
        self.dt = 1.0 / rate_hz
        self.timer = self.create_timer(self.dt, self.publish_imu_data)

        # Simulation time
        self.sim_time = 0.0

        # Send initial reset with velocity (create one-shot timer)
        self.initial_reset_timer = self.create_timer(0.5, self.send_initial_reset)
        
        # Add parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().info(
            f'IMU Simulator started: {rate_hz} Hz, initial_velocity={self.initial_velocity} m/s'
        )
        self.get_logger().info(
            f'Initial position: ({self.initial_position_x}, {self.initial_position_y}), alpha={self.initial_alpha} rad'
        )
        self.log_phase_info()

    def build_acceleration_intervals(self):
        """
        Build acceleration intervals from path parameters.
        Calculates required accelerations to transition between velocity setpoints.
        
        Returns:
            List of (start_time, end_time, type, coeffs_x, coeffs_y, omega, target_vx, target_vy) tuples
        """
        intervals = []
        
        # Ensure path arrays have same length
        n = min(len(self.path_times), len(self.path_velocities_x), 
                len(self.path_velocities_y), len(self.path_rotations))
        
        for i in range(n - 1):
            t_start = self.path_times[i]
            t_end = self.path_times[i + 1]
            dt = t_end - t_start
            
            # Target velocities for this interval (use current phase, not next!)
            v_x_target = self.path_velocities_x[i]
            v_y_target = self.path_velocities_y[i]
            omega_target = self.path_rotations[i]
            
            # Store target velocities to force-set at phase boundaries
            # The actual acceleration will be calculated dynamically based on current velocity
            intervals.append((t_start, t_end, 'constant', v_x_target, v_y_target, omega_target))
        
        return intervals
    
    def log_phase_info(self):
        """Log information about the simulation phases"""
        self.get_logger().info('========================================')
        self.get_logger().info('IMU Simulation Profile (Custom Path):')
        
        for i in range(len(self.path_times) - 1):
            t_start = self.path_times[i]
            t_end = self.path_times[i + 1]
            vx = self.path_velocities_x[i]
            vy = self.path_velocities_y[i]
            omega = self.path_rotations[i]
            
            self.get_logger().info(
                f'  Phase {i+1} ({t_start:.1f}-{t_end:.1f}s): '
                f'vx={vx:.2f} m/s, vy={vy:.2f} m/s, ω={omega:.2f} rad/s'
            )
        
        self.get_logger().info('========================================')

    def send_initial_reset(self):
        """Send initial position reset with velocity"""
        reset_msg = PositionData()
        reset_msg.x = self.initial_position_x
        reset_msg.y = self.initial_position_y
        reset_msg.z = 0.0
        reset_msg.alpha = self.initial_alpha
        reset_msg.initial_vx = self.initial_velocity
        reset_msg.initial_vy = 0.0
        self.reset_pub.publish(reset_msg)
        self.get_logger().info(
            f'Initial reset sent: position=({self.initial_position_x},{self.initial_position_y}), '
            f'alpha={self.initial_alpha} rad, velocity=({self.initial_velocity}, 0) m/s'
        )
        # Destroy the one-shot timer
        self.initial_reset_timer.cancel()
        self.destroy_timer(self.initial_reset_timer)
    
    def reset_position(self):
        """Reset position to current initial values by publishing to /position/corrected"""
        reset_msg = PositionData()
        reset_msg.x = self.initial_position_x
        reset_msg.y = self.initial_position_y
        reset_msg.z = 0.0
        reset_msg.alpha = self.initial_alpha
        reset_msg.initial_vx = self.initial_velocity
        reset_msg.initial_vy = 0.0
        self.reset_pub.publish(reset_msg)
        self.get_logger().info(
            f'Position reset: ({self.initial_position_x},{self.initial_position_y}), '
            f'velocity=({self.initial_velocity}, 0) m/s'
        )
    
    def on_parameter_change(self, params):
        """Handle runtime parameter changes"""
        from rcl_interfaces.msg import SetParametersResult
        
        rebuild_intervals = False
        
        for param in params:
            if param.name == 'initial_velocity':
                self.initial_velocity = param.value
                self.current_velocity_x = param.value
                self.get_logger().info(f'Initial velocity changed to: {self.initial_velocity} m/s')
                # Reset simulation and position
                self.sim_time = 0.0
                self.reset_position()
            elif param.name == 'initial_position_x':
                self.initial_position_x = param.value
                self.get_logger().info(f'Initial position X changed to: {self.initial_position_x} m')
            elif param.name == 'initial_position_y':
                self.initial_position_y = param.value
                self.get_logger().info(f'Initial position Y changed to: {self.initial_position_y} m')
            elif param.name == 'initial_alpha':
                self.initial_alpha = param.value
                self.get_logger().info(f'Initial alpha changed to: {self.initial_alpha} rad')
            elif param.name == 'noise_std':
                self.noise_std = param.value
                self.get_logger().info(f'Noise std changed to: {self.noise_std} m/s²')
            elif param.name == 'cycle_time':
                self.cycle_time = param.value
                self.get_logger().info(f'Cycle time changed to: {self.cycle_time} s')
            elif param.name == 'path_times':
                self.path_times = param.value
                self.get_logger().info(f'Path times updated')
                rebuild_intervals = True
            elif param.name == 'path_velocities_x':
                self.path_velocities_x = param.value
                self.get_logger().info(f'Path X velocities updated')
                rebuild_intervals = True
            elif param.name == 'path_velocities_y':
                self.path_velocities_y = param.value
                self.get_logger().info(f'Path Y velocities updated')
                rebuild_intervals = True
            elif param.name == 'path_rotations':
                self.path_rotations = param.value
                self.get_logger().info(f'Path rotations updated')
                rebuild_intervals = True
        
        # Rebuild acceleration intervals if path changed
        if rebuild_intervals:
            self.acceleration_intervals = self.build_acceleration_intervals()
            self.log_phase_info()
            self.sim_time = 0.0
            self.reset_position()
                
        return SetParametersResult(successful=True)

    def evaluate_motion(self, t):
        """
        Evaluate acceleration and angular velocity at time t using defined intervals.
        
        Args:
            t: Current time (cycles based on cycle_time parameter)

        Returns:
            Tuple of (ax, ay, omega) - linear accelerations and angular velocity
        """
        for start, end, poly_type, target_vx, target_vy, omega in self.acceleration_intervals:
            if start <= t <= end:
                dt_interval = end - start
                
                # Calculate acceleration needed to reach target velocity from current velocity
                # Use a reasonable transition time (1 second or the interval duration, whichever is shorter)
                transition_time = min(1.0, dt_interval)
                
                if transition_time > 0:
                    ax = (target_vx - self.current_velocity_x) / transition_time
                    ay = (target_vy - self.current_velocity_y) / transition_time
                    
                    # Limit acceleration to reasonable values
                    max_accel = 2.0  # m/s²
                    ax = max(-max_accel, min(max_accel, ax))
                    ay = max(-max_accel, min(max_accel, ay))
                else:
                    ax = 0.0
                    ay = 0.0
                
                return ax, ay, omega

        # Default: return 0 outside intervals
        return 0.0, 0.0, 0.0

    def get_current_phase(self, t):
        """Get the current phase number based on time"""
        for i in range(len(self.path_times) - 1):
            if self.path_times[i] <= t < self.path_times[i + 1]:
                vx = self.path_velocities_x[i]
                vy = self.path_velocities_y[i]
                omega = self.path_rotations[i]
                desc = f"vx={vx:.2f} vy={vy:.2f} ω={omega:.2f}"
                return i + 1, desc
        return 0, "Unknown"

    def publish_joint_state(self):
        """Publish joint states so robot_state_publisher can rotate wheel links."""
        # Calculate mecanum wheel velocities from robot velocity
        # Using mecanum forward kinematics
        vx = self.current_velocity_x
        vy = self.current_velocity_y
        omega = self.current_omega
        
        # Robot geometry
        lx = self.wheel_base_x / 2.0
        ly = self.wheel_base_y / 2.0
        k = lx + ly
        
        # Mecanum forward kinematics (robot velocity -> wheel velocities)
        # w = v / r for each wheel
        v1 = (vx - vy - k * omega) / self.wheel_radius  # FL
        v2 = (vx + vy + k * omega) / self.wheel_radius  # FR
        v3 = (vx - vy + k * omega) / self.wheel_radius  # RR
        v4 = (vx + vy - k * omega) / self.wheel_radius  # RL
        
        # Update wheel angles
        self.wheel_angles[0] += v1 * self.dt
        self.wheel_angles[1] += v2 * self.dt
        self.wheel_angles[2] += v3 * self.dt
        self.wheel_angles[3] += v4 * self.dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rr_joint", "wheel_rl_joint"]
        js.position = [
            self.wheel_angles[0],
            self.wheel_angles[1],
            self.wheel_angles[2],
            self.wheel_angles[3],
        ]
        js.velocity = [v1, v2, v3, v4]
        self.joint_state_pub.publish(js)

    def publish_imu_data(self):
        """
        Publish IMU data with acceleration based on current phase
        """
        # Cycle time based on cycle_time parameter
        cycle_time = self.sim_time % self.cycle_time

        # Get acceleration and angular velocity for current phase
        ax, ay, omega = self.evaluate_motion(cycle_time)
        az = 0.0  # No vertical acceleration (2D motion)

        # Add small noise to simulate real IMU
        ax += random.gauss(0, self.noise_std)
        ay += random.gauss(0, self.noise_std)

        # Get current timestamp
        current_time = self.get_clock().now().to_msg()

        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'base_link'

        # Linear acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        # Angular velocity
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = omega

        # Orientation covariance (unknown)
        imu_msg.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        # Also publish custom AccelerationData message
        accel_msg = AccelerationData()
        accel_msg.header.stamp = current_time
        accel_msg.header.frame_id = 'base_link'
        accel_msg.linear_x = ax
        accel_msg.linear_y = ay
        accel_msg.linear_z = az
        accel_msg.angular_z = omega

        self.accel_pub.publish(accel_msg)
        
        # Update current velocities based on acceleration (for wheel visualization)
        self.current_velocity_x += ax * self.dt
        self.current_velocity_y += ay * self.dt
        self.current_omega = omega  # Angular velocity is set directly from path
        
        # Publish joint states for wheel visualization
        self.publish_joint_state()

        # Log every 2 seconds
        if int(self.sim_time * 10) % 20 == 0 and self.sim_time > 0:
            phase, desc = self.get_current_phase(cycle_time)
            self.get_logger().info(
                f'[t={cycle_time:.1f}s] Phase {phase}: {desc} | '
                f'a=({ax:.3f},{ay:.3f}) m/s² | v=({self.current_velocity_x:.3f},{self.current_velocity_y:.3f}) m/s'
            )

        # Update simulation time
        self.sim_time += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = IMUSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
