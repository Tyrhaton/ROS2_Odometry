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
- "accel_profile": Use YAML acceleration profile with integration

Polynomials (same as sensor_data_simulator):
- CONSTANT: f(t) = c
- LINEAR: f(t) = a*t + b  
- QUADRATIC: f(t) = a*t² + b*t + c
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from odometry_interfaces_pkg.msg import PositionData
import math
import os
import random

import yaml
from ament_index_python.packages import get_package_share_directory


class WheelEncoderSimulator(Node):
    def __init__(self):
        super().__init__('wheel_encoder_simulator')

        # Declare parameters
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('forward_velocity', 5.0)  # rad/s base velocity
        self.declare_parameter('rotation_velocity', 1.0)  # rad/s rotation component
        self.declare_parameter('noise_std', 0.1)  # rad/s standard deviation
        self.declare_parameter('motion_mode', 'circle')  # forward, rotation, diagonal, circle, polynomial
        self.declare_parameter('publish_time_marker', True)
        self.declare_parameter('segment_marker_offset_x', 0.0)
        self.declare_parameter('segment_marker_offset_y', -0.7)
        self.declare_parameter('segment_marker_offset_z', 0.8)
        self.declare_parameter('segment_marker_scale', 0.1)
        self.declare_parameter('time_marker_offset_x', 0.0)
        self.declare_parameter('time_marker_offset_y', -0.6)
        self.declare_parameter('time_marker_offset_z', 0.68)
        self.declare_parameter('time_marker_scale', 0.1)
        self.declare_parameter('distance_marker_offset_x', 0.0)
        self.declare_parameter('distance_marker_offset_y', -0.5)
        self.declare_parameter('distance_marker_offset_z', 0.56)
        self.declare_parameter('distance_marker_scale', 0.1)
        self.declare_parameter('speed_marker_offset_x', 0.0)
        self.declare_parameter('speed_marker_offset_y', -0.4)
        self.declare_parameter('speed_marker_offset_z', 0.44)
        self.declare_parameter('speed_marker_scale', 0.1)
        self.declare_parameter('accel_marker_offset_x', 0.0)
        self.declare_parameter('accel_marker_offset_y', -0.3)
        self.declare_parameter('accel_marker_offset_z', 0.32)
        self.declare_parameter('accel_marker_scale', 0.1)
        default_profile_path = os.path.join(
            get_package_share_directory('odometry_pkg'),
            'config',
            'accel_profile.yaml'
        )
        self.declare_parameter('accel_profile_path', default_profile_path)

        # Get parameters
        rate_hz = self.get_parameter('publish_rate_hz').value
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.rotation_vel = self.get_parameter('rotation_velocity').value
        self.noise_std = self.get_parameter('noise_std').value
        self.motion_mode = self.get_parameter('motion_mode').value
        self.publish_time_marker_enabled = self.get_parameter('publish_time_marker').value
        self.segment_marker_offset_x = self.get_parameter('segment_marker_offset_x').value
        self.segment_marker_offset_y = self.get_parameter('segment_marker_offset_y').value
        self.segment_marker_offset_z = self.get_parameter('segment_marker_offset_z').value
        self.segment_marker_scale = self.get_parameter('segment_marker_scale').value
        self.time_marker_offset_x = self.get_parameter('time_marker_offset_x').value
        self.time_marker_offset_y = self.get_parameter('time_marker_offset_y').value
        self.time_marker_offset_z = self.get_parameter('time_marker_offset_z').value
        self.time_marker_scale = self.get_parameter('time_marker_scale').value
        self.distance_marker_offset_x = self.get_parameter('distance_marker_offset_x').value
        self.distance_marker_offset_y = self.get_parameter('distance_marker_offset_y').value
        self.distance_marker_offset_z = self.get_parameter('distance_marker_offset_z').value
        self.distance_marker_scale = self.get_parameter('distance_marker_scale').value
        self.speed_marker_offset_x = self.get_parameter('speed_marker_offset_x').value
        self.speed_marker_offset_y = self.get_parameter('speed_marker_offset_y').value
        self.speed_marker_offset_z = self.get_parameter('speed_marker_offset_z').value
        self.speed_marker_scale = self.get_parameter('speed_marker_scale').value
        self.accel_marker_offset_x = self.get_parameter('accel_marker_offset_x').value
        self.accel_marker_offset_y = self.get_parameter('accel_marker_offset_y').value
        self.accel_marker_offset_z = self.get_parameter('accel_marker_offset_z').value
        self.accel_marker_scale = self.get_parameter('accel_marker_scale').value
        self.accel_profile_path = self.get_parameter('accel_profile_path').value

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
        self.time_marker_pub = self.create_publisher(
            Marker,
            '/simulation/time_marker',
            1
        )
        self.distance_marker_pub = self.create_publisher(
            Marker,
            '/simulation/distance_marker',
            1
        )
        self.speed_marker_pub = self.create_publisher(
            Marker,
            '/simulation/speed_marker',
            1
        )
        self.accel_marker_pub = self.create_publisher(
            Marker,
            '/simulation/accel_marker',
            1
        )
        self.segment_marker_pub = self.create_publisher(
            Marker,
            '/simulation/segment_marker',
            1
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
        self.startup_joint_state_skip = int(0.5 / self.dt)
        self.startup_joint_state_ticks = 50
        self.startup_joint_timer = self.create_timer(
            self.dt,
            self.publish_startup_joint_state
        )

        # Simulation time
        self.sim_time = 0.0

        # Track wheel angles for joint state publishing
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RR, RL

        # Load acceleration profile for scripted motion
        self.profile_segments = []
        self.profile_loop = True
        self.profile_total_time = None
        self.reset_profile_state()
        self.load_accel_profile()
        
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
            'polynomial': 'Polynomial intervals -> configurable velocity profile',
            'accel_profile': 'Acceleration profile -> integrated wheel motion'
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
                # Reset any profile integration state
                self.reset_profile_state()
                # Reset position to origin when mode changes
                self.reset_position()
            elif param.name == 'forward_velocity':
                self.forward_vel = param.value
            elif param.name == 'rotation_velocity':
                self.rotation_vel = param.value
            elif param.name == 'accel_profile_path':
                self.accel_profile_path = param.value
                self.load_accel_profile()
                
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

    def load_accel_profile(self):
        """Load an acceleration profile from YAML for scripted motion."""
        self.profile_segments = []
        self.profile_loop = True
        try:
            with open(self.accel_profile_path, 'r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warn(
                f'Failed to load accel profile "{self.accel_profile_path}": {exc}'
            )
            self.reset_profile_state()
            return

        profile = data.get('accel_profile', data)
        self.profile_loop = bool(profile.get('loop', True))
        total_time = profile.get('total_time')
        total_distance = profile.get('total_distance')
        segments = profile.get('segments', [])

        raw_segments = []
        for index, segment in enumerate(segments):
            duration = segment.get('duration')
            if duration is not None:
                duration = float(duration)
            ax = segment.get('ax')
            if ax is not None:
                ax = float(ax)
            ay = segment.get('ay')
            if ay is not None:
                ay = float(ay)
            rotation = segment.get('rotation')
            if rotation is None:
                rotation = segment.get('alpha')
            if rotation is not None:
                rotation = float(rotation)
            raw_segments.append({
                'duration': duration,
                'ax': ax,
                'ay': ay,
                'rotation': rotation,
                'index': index,
            })

        missing_durations = [seg for seg in raw_segments if seg['duration'] is None]
        if missing_durations:
            if total_time is not None:
                total_time = float(total_time)
                self.profile_total_time = total_time
                known_time = sum(
                    seg['duration'] for seg in raw_segments if seg['duration'] is not None
                )
                remaining = total_time - known_time
                if remaining < 0.0:
                    self.get_logger().warn(
                        'total_time is less than known segment durations; '
                        'missing durations set to 0.'
                    )
                    remaining = 0.0
                fill = remaining / len(missing_durations)
                for seg in missing_durations:
                    seg['duration'] = fill
            else:
                self.get_logger().warn(
                    'Segments missing duration but total_time is not set; '
                    'missing durations set to 0.'
                )
                for seg in missing_durations:
                    seg['duration'] = 0.0
        elif total_time is not None:
            self.profile_total_time = float(total_time)

        valid_segments = []
        for seg in raw_segments:
            if seg['duration'] is None or seg['duration'] <= 0.0:
                self.get_logger().warn(
                    f"Skipping segment {seg['index']}: duration must be > 0"
                )
                continue
            if seg['ax'] is None:
                seg['ax'] = None
            if seg['ay'] is None:
                seg['ay'] = 0.0
            if seg['rotation'] is None:
                seg['rotation'] = 0.0
            valid_segments.append(seg)

        def compute_distance(segments, ax_missing_value=None, ax_scale=1.0):
            vx = 0.0
            distance = 0.0
            for seg in segments:
                ax_value = seg['ax']
                if ax_value is None and ax_missing_value is not None:
                    ax_value = ax_missing_value
                if ax_value is None:
                    ax_value = 0.0
                ax_value *= ax_scale
                dt = seg['duration']
                distance += vx * dt + 0.5 * ax_value * dt * dt
                vx += ax_value * dt
            return distance

        missing_ax = [seg for seg in valid_segments if seg['ax'] is None]
        if missing_ax:
            if total_distance is None:
                self.get_logger().warn(
                    'Segments missing ax but total_distance is not set; '
                    'missing ax set to 0.'
                )
                for seg in missing_ax:
                    seg['ax'] = 0.0
            else:
                total_distance = float(total_distance)
                base_distance = compute_distance(valid_segments, ax_missing_value=0.0)
                slope = compute_distance(valid_segments, ax_missing_value=1.0) - base_distance
                if abs(slope) < 1e-9:
                    self.get_logger().warn(
                        'Unable to solve for missing ax (slope ~ 0); '
                        'missing ax set to 0.'
                    )
                    for seg in missing_ax:
                        seg['ax'] = 0.0
                else:
                    ax_fill = (total_distance - base_distance) / slope
                    for seg in missing_ax:
                        seg['ax'] = ax_fill

        if total_distance is not None and not missing_ax:
            total_distance = float(total_distance)
            base_distance = compute_distance(valid_segments)
            if abs(base_distance) < 1e-9:
                self.get_logger().warn(
                    'total_distance set but computed distance is 0; '
                    'ax values left unchanged.'
                )
            else:
                scale = total_distance / base_distance
                for seg in valid_segments:
                    seg['ax'] *= scale

        for seg in valid_segments:
            self.profile_segments.append({
                'duration': seg['duration'],
                'ax': seg['ax'],
                'ay': seg['ay'],
                'alpha': seg['rotation'],
            })

        if not self.profile_segments:
            self.get_logger().warn('Acceleration profile has no valid segments.')
        else:
            if self.profile_total_time is None:
                self.profile_total_time = sum(seg['duration'] for seg in self.profile_segments)
            self.get_logger().info(
                f'Loaded accel profile with {len(self.profile_segments)} segments '
                f'(loop={self.profile_loop})'
            )

        self.reset_profile_state()

    def reset_profile_state(self):
        """Reset integration state for the acceleration profile."""
        self.profile_index = 0
        self.profile_elapsed = 0.0
        self.profile_vx = 0.0
        self.profile_vy = 0.0
        self.profile_omega = 0.0
        self.profile_distance = 0.0
        self.profile_ax = 0.0
        self.profile_ay = 0.0
        self.profile_active = True

    def step_accel_profile(self):
        """Integrate the current acceleration segment and return robot velocities."""
        if not self.profile_segments:
            return 0.0, 0.0, 0.0

        if not self.profile_active:
            return self.profile_vx, self.profile_vy, self.profile_omega

        segment = self.profile_segments[self.profile_index]
        ax = segment['ax']
        ay = segment['ay']
        alpha = segment['alpha']
        self.profile_ax = ax
        self.profile_ay = ay

        self.profile_vx += ax * self.dt
        self.profile_vy += ay * self.dt
        self.profile_omega += alpha * self.dt
        step_distance = math.hypot(self.profile_vx * self.dt, self.profile_vy * self.dt)
        self.profile_distance += step_distance

        self.profile_elapsed += self.dt
        if self.profile_elapsed >= segment['duration']:
            self.profile_elapsed -= segment['duration']
            self.profile_index += 1
            if self.profile_index >= len(self.profile_segments):
                if self.profile_loop:
                    self.profile_index = 0
                else:
                    self.profile_index = len(self.profile_segments) - 1
                    self.profile_vx = 0.0
                    self.profile_vy = 0.0
                    self.profile_omega = 0.0
                    self.profile_active = False

        return self.profile_vx, self.profile_vy, self.profile_omega

    def compute_wheel_velocities(self, vx, vy, omega):
        """Compute mecanum wheel velocities from robot velocities."""
        wheel_radius = 0.05  # meters
        lx = 0.15  # half length (wheel_base_x / 2)
        ly = 0.125  # half width (wheel_base_y / 2)
        k = lx + ly  # = 0.275m

        v1 = (vx - vy - k * omega) / wheel_radius  # FL
        v2 = (vx + vy + k * omega) / wheel_radius  # FR
        v3 = (vx - vy + k * omega) / wheel_radius  # RR
        v4 = (vx + vy - k * omega) / wheel_radius  # RL
        return v1, v2, v3, v4

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
            use_robot_velocities = False
        elif self.motion_mode == 'accel_profile':
            vx, vy, omega = self.step_accel_profile()
            use_robot_velocities = True
        elif self.motion_mode == 'forward':
            # Pure forward motion - all wheels same speed
            forward_vel = self.forward_vel
            rotation_vel = 0.0
            use_robot_velocities = False
        elif self.motion_mode == 'rotation':
            # Pure rotation - left wheels opposite to right
            forward_vel = 0.0
            rotation_vel = self.rotation_vel
            use_robot_velocities = False
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
            use_robot_velocities = False

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
        # For circle mode:
        # - forward_vel is the base wheel speed in rad/s
        # - rotation_vel controls how tight the circle is
        if not use_robot_velocities:
            wheel_radius = 0.05  # meters
            vx = forward_vel * wheel_radius  # Linear velocity in m/s (e.g., 5.0 * 0.05 = 0.25 m/s)
            omega = rotation_vel * 0.5  # Angular velocity in rad/s (e.g., 1.0 * 0.5 = 0.5 rad/s)
            vy = 0.0  # No strafing for circle mode
        
        # Circle radius = vx / omega = 0.25 / 0.5 = 0.5 meters
        # Full circle time = 2*pi / omega = 2*pi / 0.5 = 12.6 seconds
        
        # Apply CORRECT mecanum forward kinematics
        # These equations when inverted give back the correct vx, vy, omega
        v1, v2, v3, v4 = self.compute_wheel_velocities(vx, vy, omega)

        # Add realistic noise to simulate encoder measurement errors
        noise_std = self.noise_std
        if self.motion_mode == 'accel_profile':
            noise_std = 0.0
        v1 += random.gauss(0, noise_std)
        v2 += random.gauss(0, noise_std)
        v3 += random.gauss(0, noise_std)
        v4 += random.gauss(0, noise_std)

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
        self.publish_time_marker()

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

    def publish_startup_joint_state(self):
        """Publish zero joint states briefly so wheel links appear immediately."""
        if self.startup_joint_state_skip > 0:
            self.startup_joint_state_skip -= 1
            return
        if self.startup_joint_state_ticks <= 0:
            self.startup_joint_timer.cancel()
            return
        self.startup_joint_state_ticks -= 1
        self.publish_joint_state([0.0, 0.0, 0.0, 0.0])

    def publish_time_marker(self):
        """Publish a text marker with elapsed simulation time for RViz."""
        if not self.publish_time_marker_enabled:
            return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.ns = "simulation_time"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.time_marker_offset_x
        marker.pose.position.y = self.time_marker_offset_y
        marker.pose.position.z = self.time_marker_offset_z
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.time_marker_scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        display_time = self.sim_time
        if (self.motion_mode == 'accel_profile'
                and not self.profile_loop
                and not self.profile_active
                and self.profile_total_time is not None):
            display_time = self.profile_total_time
        marker.text = f"time:{display_time:.2f}s"
        self.time_marker_pub.publish(marker)

        distance_marker = Marker()
        distance_marker.header.stamp = marker.header.stamp
        distance_marker.header.frame_id = marker.header.frame_id
        distance_marker.ns = "simulation_distance"
        distance_marker.id = 1
        distance_marker.type = Marker.TEXT_VIEW_FACING
        distance_marker.action = Marker.ADD
        distance_marker.pose.position.x = self.distance_marker_offset_x
        distance_marker.pose.position.y = self.distance_marker_offset_y
        distance_marker.pose.position.z = self.distance_marker_offset_z
        distance_marker.pose.orientation.w = 1.0
        distance_marker.scale.z = self.distance_marker_scale
        distance_marker.color.r = 1.0
        distance_marker.color.g = 1.0
        distance_marker.color.b = 1.0
        distance_marker.color.a = 1.0
        distance_marker.text = f"dist:{self.profile_distance:.2f}m"
        self.distance_marker_pub.publish(distance_marker)

        speed_marker = Marker()
        speed_marker.header.stamp = marker.header.stamp
        speed_marker.header.frame_id = marker.header.frame_id
        speed_marker.ns = "simulation_speed"
        speed_marker.id = 2
        speed_marker.type = Marker.TEXT_VIEW_FACING
        speed_marker.action = Marker.ADD
        speed_marker.pose.position.x = self.speed_marker_offset_x
        speed_marker.pose.position.y = self.speed_marker_offset_y
        speed_marker.pose.position.z = self.speed_marker_offset_z
        speed_marker.pose.orientation.w = 1.0
        speed_marker.scale.z = self.speed_marker_scale
        speed_marker.color.r = 1.0
        speed_marker.color.g = 1.0
        speed_marker.color.b = 1.0
        speed_marker.color.a = 1.0
        speed = math.hypot(self.profile_vx, self.profile_vy)
        speed_marker.text = f"speed:{speed:.2f}m/s"
        self.speed_marker_pub.publish(speed_marker)

        accel_marker = Marker()
        accel_marker.header.stamp = marker.header.stamp
        accel_marker.header.frame_id = marker.header.frame_id
        accel_marker.ns = "simulation_accel"
        accel_marker.id = 3
        accel_marker.type = Marker.TEXT_VIEW_FACING
        accel_marker.action = Marker.ADD
        accel_marker.pose.position.x = self.accel_marker_offset_x
        accel_marker.pose.position.y = self.accel_marker_offset_y
        accel_marker.pose.position.z = self.accel_marker_offset_z
        accel_marker.pose.orientation.w = 1.0
        accel_marker.scale.z = self.accel_marker_scale
        accel_marker.color.r = 1.0
        accel_marker.color.g = 1.0
        accel_marker.color.b = 1.0
        accel_marker.color.a = 1.0
        accel = math.hypot(self.profile_ax, self.profile_ay)
        accel_marker.text = f"accel:{accel:.2f}m/s^2"
        self.accel_marker_pub.publish(accel_marker)

        segment_marker = Marker()
        segment_marker.header.stamp = marker.header.stamp
        segment_marker.header.frame_id = marker.header.frame_id
        segment_marker.ns = "simulation_segment"
        segment_marker.id = 4
        segment_marker.type = Marker.TEXT_VIEW_FACING
        segment_marker.action = Marker.ADD
        segment_marker.pose.position.x = self.segment_marker_offset_x
        segment_marker.pose.position.y = self.segment_marker_offset_y
        segment_marker.pose.position.z = self.segment_marker_offset_z
        segment_marker.pose.orientation.w = 1.0
        segment_marker.scale.z = self.segment_marker_scale
        segment_marker.color.r = 1.0
        segment_marker.color.g = 1.0
        segment_marker.color.b = 1.0
        segment_marker.color.a = 1.0
        segment_index = self.profile_index + 1 if self.profile_segments else 0
        segment_total = len(self.profile_segments)
        segment_marker.text = f"seg:{segment_index}/{segment_total}"
        self.segment_marker_pub.publish(segment_marker)


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
