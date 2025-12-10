#!/usr/bin/env python3
"""
Wheel encoder simulator for mecanum wheel odometry testing

Publishes simulated wheel velocities in rad/s for 4 mecanum wheels.
Supports polynomial interpolation (constant, linear, quadratic) over
configurable time intervals as per Assignment 4 requirements.

Modes:
- "forward": All wheels same direction (straight forward)
- "rotation": Rotate in place (left wheels opposite to right)
- "diagonal": Strafe sideways (mecanum-specific motion)
- "circle": Circular motion (forward + rotation)
- "polynomial": Use polynomial intervals for velocity

Polynomials (same as sensor_data_simulator):
- CONSTANT: f(t) = c
- LINEAR: f(t) = a*t + b  
- QUADRATIC: f(t) = a*t² + b*t + c
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from odometry_interfaces_pkg.msg import PositionData
import math
import random


class WheelEncoderSimulator(Node):
    def __init__(self):
        super().__init__('wheel_encoder_simulator')

        # Declare parameters
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('forward_velocity', 5.0)  # rad/s base velocity
        self.declare_parameter('rotation_velocity', 1.0)  # rad/s rotation component
        self.declare_parameter('noise_std', 0.1)  # rad/s standard deviation
        self.declare_parameter('motion_mode', 'circle')  # forward, rotation, diagonal, circle, polynomial

        # Get parameters
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.rotation_vel = self.get_parameter('rotation_velocity').value
        self.noise_std = self.get_parameter('noise_std').value
        self.motion_mode = self.get_parameter('motion_mode').value

        # Define polynomial intervals for velocity (as per Assignment 4)
        # Format: list of (start_time, end_time, type, coefficients)
        # type: 'constant', 'linear', 'quadratic'
        # coefficients: [c] for constant, [a, b] for linear (a*t+b), [a, b, c] for quadratic (a*t²+b*t+c)
        # Note: For linear/quadratic, time is relative to interval start
        self.velocity_intervals = [
            # Interval 1: 0-10s - Constant forward velocity
            (0.0, 10.0, 'constant', [3.0]),  # v = 3.0 rad/s constant
            # Interval 2: 10-20s - Linear acceleration
            (10.0, 20.0, 'linear', [0.5, 3.0]),  # v = 0.5*(t-10) + 3.0 -> 3.0 to 8.0
            # Interval 3: 20-30s - Constant high speed
            (20.0, 30.0, 'constant', [8.0]),  # v = 8.0 rad/s
            # Interval 4: 30-45s - Quadratic deceleration
            (30.0, 45.0, 'quadratic', [-0.04, 0.0, 8.0]),  # v starts at 8.0, slows down
            # Interval 5: 45-60s - Cycle repeats (linear ramp)
            (45.0, 60.0, 'linear', [0.2, 0.0]),  # v = 0.2*(t-45) -> 0 to 3.0
        ]
        
        # Rotation intervals (independent control of rotation)
        self.rotation_intervals = [
            # Interval 1: 0-15s - No rotation (straight)
            (0.0, 15.0, 'constant', [0.0]),
            # Interval 2: 15-25s - Constant right turn
            (15.0, 25.0, 'constant', [0.5]),  # omega = 0.5 rad/s
            # Interval 3: 25-35s - Constant left turn
            (25.0, 35.0, 'constant', [-0.5]),  # omega = -0.5 rad/s
            # Interval 4: 35-50s - Quadratic turn (increasing)
            (35.0, 50.0, 'quadratic', [0.01, 0.0, 0.0]),  # omega increases quadratically
            # Interval 5: 50-60s - Back to straight
            (50.0, 60.0, 'constant', [0.0]),
        ]

        # Publisher for wheel velocities
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/wheel_encoders/velocities',
            10
        )

        # Joint state publisher so robot_state_publisher can rotate wheel TFs
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Publisher for position reset (when mode changes)
        self.position_reset_pub = self.create_publisher(
            PositionData,
            '/position/corrected',
            10
        )

        # Timer
        self.dt = 1.0 / rate_hz
        self.timer = self.create_timer(self.dt, self.publish_velocities)

        # Simulation time
        self.sim_time = 0.0

        # Track wheel angles for joint state publishing
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RR, RL
        
        # Add parameter callback for runtime mode switching
        self.add_on_set_parameters_callback(self.on_parameter_change)

        self.get_logger().info(
            f'Wheel Encoder Simulator started: {rate_hz} Hz, mode={self.motion_mode}'
        )
        self.get_logger().info(
            f'Base velocities: forward={self.forward_vel} rad/s, rotation={self.rotation_vel} rad/s'
        )
        self.log_mode_info()
    
    def log_mode_info(self):
        """Log information about current motion mode"""
        mode_descriptions = {
            'forward': 'All wheels same speed -> straight forward motion',
            'rotation': 'Left/right wheels opposite -> rotate in place',
            'diagonal': 'Diagonal wheel pairs -> strafe sideways (mecanum)',
            'circle': 'Forward + rotation -> circular motion',
            'polynomial': 'Polynomial intervals -> configurable velocity profile'
        }
        desc = mode_descriptions.get(self.motion_mode, f'Unknown mode! Valid modes: {list(mode_descriptions.keys())}')
        self.get_logger().info(f'Motion mode: {self.motion_mode} - {desc}')
    
    def on_parameter_change(self, params):
        """Handle runtime parameter changes"""
        from rcl_interfaces.msg import SetParametersResult
        
        for param in params:
            if param.name == 'motion_mode':
                old_mode = self.motion_mode
                self.motion_mode = param.value
                self.get_logger().info(f'Motion mode changed: {old_mode} -> {self.motion_mode}')
                self.log_mode_info()
                # Reset simulation time on mode change
                self.sim_time = 0.0
                # Reset position to origin when mode changes
                self.reset_position()
            elif param.name == 'forward_velocity':
                self.forward_vel = param.value
            elif param.name == 'rotation_velocity':
                self.rotation_vel = param.value
                
        return SetParametersResult(successful=True)
    
    def reset_position(self):
        """Reset position to origin by publishing to /position/corrected"""
        reset_msg = PositionData()
        reset_msg.x = 0.0
        reset_msg.y = 0.0
        reset_msg.z = 0.0
        reset_msg.alpha = 0.0  # Reset orientation too
        reset_msg.initial_vx = 0.0
        reset_msg.initial_vy = 0.0
        self.position_reset_pub.publish(reset_msg)
        self.get_logger().info('Position reset to origin (0, 0, α=0)')
    
    def evaluate_polynomial(self, t, intervals):
        """
        Evaluate polynomial value at time t using defined intervals.
        Returns 0.0 if t is outside all intervals (as per Assignment 4 requirement).
        
        Args:
            t: Current time
            intervals: List of (start, end, type, coeffs) tuples
        
        Returns:
            Polynomial value at time t
        """
        for start, end, poly_type, coeffs in intervals:
            if start <= t <= end:
                # Time relative to interval start
                t_rel = t - start
                
                if poly_type == 'constant':
                    return coeffs[0]
                elif poly_type == 'linear':
                    # f(t) = a*t + b
                    a, b = coeffs[0], coeffs[1] if len(coeffs) > 1 else 0.0
                    return a * t_rel + b
                elif poly_type == 'quadratic':
                    # f(t) = a*t² + b*t + c
                    a = coeffs[0]
                    b = coeffs[1] if len(coeffs) > 1 else 0.0
                    c = coeffs[2] if len(coeffs) > 2 else 0.0
                    return a * t_rel * t_rel + b * t_rel + c
        
        # Default: return 0 outside intervals (Assignment 4 requirement)
        return 0.0

    def publish_velocities(self):
        """
        Publish wheel velocities in rad/s

        Wheel numbering (mecanum configuration):
        1 = front-left  (FL)
        2 = front-right (FR)
        3 = rear-right  (RR)
        4 = rear-left   (RL)

        Mecanum forward kinematics (robot velocity -> wheel velocities):
        w1 = (1/r) * (vx - vy - (lx+ly)*omega)  # FL
        w2 = (1/r) * (vx + vy + (lx+ly)*omega)  # FR
        w3 = (1/r) * (vx - vy + (lx+ly)*omega)  # RR
        w4 = (1/r) * (vx + vy - (lx+ly)*omega)  # RL
        
        Simplified for testing (normalized):
        - Forward (vx only): all wheels same
        - Strafe (vy only): FL=RR opposite to FR=RL
        - Rotation (omega only): left wheels opposite to right wheels
        """
        
        # Get base velocities based on mode
        if self.motion_mode == 'polynomial':
            # Use polynomial intervals for forward and rotation velocity
            # Cycle time after 60s for continuous demo
            cycle_time = self.sim_time % 60.0
            forward_vel = self.evaluate_polynomial(cycle_time, self.velocity_intervals)
            rotation_vel = self.evaluate_polynomial(cycle_time, self.rotation_intervals)
        elif self.motion_mode == 'forward':
            # Pure forward motion - all wheels same speed
            forward_vel = self.forward_vel
            rotation_vel = 0.0
        elif self.motion_mode == 'rotation':
            # Pure rotation - left wheels opposite to right
            forward_vel = 0.0
            rotation_vel = self.rotation_vel
        elif self.motion_mode == 'diagonal':
            # Strafe motion - mecanum specific
            forward_vel = self.forward_vel
            rotation_vel = 0.0
            # For strafe: v1=v3 opposite to v2=v4
            v1 = forward_vel  # FL: positive
            v2 = -forward_vel  # FR: negative (for left strafe)
            v3 = forward_vel  # RR: positive
            v4 = -forward_vel  # RL: negative
            # Add noise and publish
            v1 += random.gauss(0, self.noise_std)
            v2 += random.gauss(0, self.noise_std)
            v3 += random.gauss(0, self.noise_std)
            v4 += random.gauss(0, self.noise_std)
            msg = Float64MultiArray()
            msg.data = [v1, v2, v3, v4]
            self.publisher.publish(msg)
            self.publish_joint_state([v1, v2, v3, v4])
            self.sim_time += self.dt
            return
        else:  # 'circle' - default
            # Circular motion - continuous circle path
            # Use constant forward velocity with constant rotation for clean circle
            forward_vel = self.forward_vel
            rotation_vel = self.rotation_vel
            
            # Only smooth acceleration at the very start (first 2 seconds)
            if self.sim_time < 2.0:
                forward_vel *= (self.sim_time / 2.0)
                rotation_vel *= (self.sim_time / 2.0)

        # Calculate mecanum wheel velocities using proper mecanum forward kinematics
        # 
        # The CORRECT forward kinematics (robot velocity -> wheel velocities) are:
        #   w1 = (1/r) * (vx - vy - (lx+ly)*omega)  # FL
        #   w2 = (1/r) * (vx + vy + (lx+ly)*omega)  # FR
        #   w3 = (1/r) * (vx - vy + (lx+ly)*omega)  # RR
        #   w4 = (1/r) * (vx + vy - (lx+ly)*omega)  # RL
        #
        # The inverse kinematics (in mecanum_position_approximator.cpp) are:
        #   vx = r/4 * (w1 + w2 + w3 + w4)
        #   vy = r/4 * (-w1 + w2 + w3 - w4)  
        #   omega = r/(4*(lx+ly)) * (-w1 + w2 - w3 + w4)
        #
        # For a CIRCLE, we need: constant vx + constant omega (and vy=0)
        # The radius of the circle will be: R = vx / omega
        
        # Robot geometry parameters (matching mecanum_position_approximator)
        wheel_radius = 0.05  # meters
        lx = 0.15  # half length (wheel_base_x / 2)
        ly = 0.125  # half width (wheel_base_y / 2)
        k = lx + ly  # = 0.275m
        
        # For circle mode:
        # - forward_vel is the base wheel speed in rad/s
        # - rotation_vel controls how tight the circle is
        vx = forward_vel * wheel_radius  # Linear velocity in m/s (e.g., 5.0 * 0.05 = 0.25 m/s)
        omega = rotation_vel * 0.5  # Angular velocity in rad/s (e.g., 1.0 * 0.5 = 0.5 rad/s)
        vy = 0.0  # No strafing for circle mode
        
        # Circle radius = vx / omega = 0.25 / 0.5 = 0.5 meters
        # Full circle time = 2*pi / omega = 2*pi / 0.5 = 12.6 seconds
        
        # Apply CORRECT mecanum forward kinematics
        # These equations when inverted give back the correct vx, vy, omega
        v1 = (vx - vy - k * omega) / wheel_radius  # FL
        v2 = (vx + vy + k * omega) / wheel_radius  # FR
        v3 = (vx - vy + k * omega) / wheel_radius  # RR
        v4 = (vx + vy - k * omega) / wheel_radius  # RL

        # Add realistic noise to simulate encoder measurement errors
        v1 += random.gauss(0, self.noise_std)
        v2 += random.gauss(0, self.noise_std)
        v3 += random.gauss(0, self.noise_std)
        v4 += random.gauss(0, self.noise_std)

        # Create and publish message
        msg = Float64MultiArray()
        msg.data = [v1, v2, v3, v4]

        self.publisher.publish(msg)
        self.publish_joint_state([v1, v2, v3, v4])
        
        # Log every 2 seconds for monitoring
        if int(self.sim_time * 10) % 20 == 0 and self.sim_time > 0:
            self.get_logger().info(
                f'[t={self.sim_time:.1f}s] Mode: {self.motion_mode} | '
                f'Wheels: [{v1:.2f}, {v2:.2f}, {v3:.2f}, {v4:.2f}] rad/s'
            )

        # Update simulation time
        self.sim_time += self.dt

    def publish_joint_state(self, wheel_vels):
        """Publish joint states so robot_state_publisher can rotate wheel links."""
        for i in range(4):
            self.wheel_angles[i] += wheel_vels[i] * self.dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rr_joint", "wheel_rl_joint"]
        js.position = [
            self.wheel_angles[0],
            self.wheel_angles[1],
            self.wheel_angles[2],
            self.wheel_angles[3],
        ]
        js.velocity = wheel_vels
        self.joint_state_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelEncoderSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
