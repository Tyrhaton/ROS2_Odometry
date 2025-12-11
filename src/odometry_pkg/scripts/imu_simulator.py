#!/usr/bin/env python3
"""
IMU Simulator - Configurable motion paths

Supports two modes:
1. Simple mode: forward_velocity, forward_duration, rotation_angle parameters
2. Custom path mode: path_times, path_velocities_x, path_velocities_y, path_rotations arrays

The simulator outputs acceleration data that integrates correctly to the target velocities.
During stopped phases, outputs exactly zero to allow ZUPT drift correction.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Imu, JointState
from odometry_interfaces_pkg.msg import AccelerationData, PositionData
import math
from dataclasses import dataclass
from typing import Tuple, List
import ast


@dataclass
class MotionState:
    """Current motion state"""
    vx: float = 0.0  # Target velocity X (body frame)
    vy: float = 0.0  # Target velocity Y (body frame)
    omega: float = 0.0  # Angular velocity
    ax: float = 0.0  # Acceleration X
    ay: float = 0.0  # Acceleration Y
    is_stopped: bool = True  # Whether robot should be completely still


class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')

        # Declare parameters
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('initial_position_x', 0.0)
        self.declare_parameter('initial_position_y', 0.0)
        self.declare_parameter('initial_alpha', 0.0)
        self.declare_parameter('cycle_time', 30.0)
        self.declare_parameter('transition_time', 0.5)  # Time for velocity transitions

        # Simple mode parameters
        self.declare_parameter('forward_velocity', 0.3)
        self.declare_parameter('forward_duration', 5.0)
        self.declare_parameter('rotation_angle', 1.5708)
        self.declare_parameter('rotation_duration', 3.0)

        # Custom path mode parameters (if provided, overrides simple mode)
        # Times are the START times of each phase
        # Use ParameterDescriptor with DOUBLE_ARRAY type for proper type inference
        double_array_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Array of double values'
        )
        self.declare_parameter('path_times', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('path_velocities_x', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('path_velocities_y', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('path_rotations', Parameter.Type.DOUBLE_ARRAY)

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_x', 0.30)
        self.declare_parameter('wheel_base_y', 0.25)

        # Get parameters
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.initial_position_x = self.get_parameter('initial_position_x').value
        self.initial_position_y = self.get_parameter('initial_position_y').value
        self.initial_alpha = self.get_parameter('initial_alpha').value
        self.cycle_time = self.get_parameter('cycle_time').value
        self.transition_time = self.get_parameter('transition_time').value

        self.forward_velocity = self.get_parameter('forward_velocity').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.rotation_duration = self.get_parameter('rotation_duration').value

        self.path_times = self._parse_array_param('path_times')
        self.path_velocities_x = self._parse_array_param('path_velocities_x')
        self.path_velocities_y = self._parse_array_param('path_velocities_y')
        self.path_rotations = self._parse_array_param('path_rotations')

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value

        # Calculate derived values
        self.dt = 1.0 / self.rate_hz

        # Check if custom path mode
        self.use_custom_path = (len(self.path_times) > 0 and
                                len(self.path_velocities_x) > 0)

        # Build timeline
        if self.use_custom_path:
            self._build_custom_timeline()
        else:
            self._build_simple_timeline()

        # State tracking
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_omega = 0.0
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.accel_pub = self.create_publisher(AccelerationData, '/simulator/acceleration', 10)
        self.reset_pub = self.create_publisher(PositionData, '/position/corrected', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.publish_imu_data)
        self.sim_time = 0.0

        # Send initial reset
        self.initial_reset_timer = self.create_timer(0.5, self.send_initial_reset)

        self.get_logger().info(f'IMU Simulator started @ {self.rate_hz} Hz')
        if self.use_custom_path:
            self.get_logger().info('Using CUSTOM PATH mode')
            self.get_logger().info(f'  path_times: {self.path_times}')
            self.get_logger().info(f'  path_velocities_x: {self.path_velocities_x}')
            self.get_logger().info(f'  path_velocities_y: {self.path_velocities_y}')
            self.get_logger().info(f'  path_rotations: {self.path_rotations}')
        else:
            self.get_logger().info('Using SIMPLE mode')
        self._log_timeline()

    def _parse_array_param(self, param_name: str) -> List[float]:
        """Parse array parameter - handles both list and string formats"""
        try:
            param = self.get_parameter(param_name)
            value = param.value
        except Exception:
            # Parameter not initialized
            return []

        # If None or not set
        if value is None:
            return []

        # If already a list, return it
        if isinstance(value, (list, tuple)):
            result = [float(x) for x in value if x is not None]
            return result

        # If string, try to parse it
        if isinstance(value, str):
            value = value.strip()
            if value == '' or value == '[]':
                return []
            try:
                parsed = ast.literal_eval(value)
                if isinstance(parsed, (list, tuple)):
                    return [float(x) for x in parsed]
            except (ValueError, SyntaxError) as e:
                self.get_logger().warn(f'Failed to parse {param_name}: {value} - {e}')
                return []

        return []

    def _build_custom_timeline(self):
        """Build timeline from custom path parameters"""
        self.timeline = []
        t_trans = self.transition_time

        # Ensure arrays have same length
        n = min(len(self.path_times), len(self.path_velocities_x))
        if len(self.path_velocities_y) < n:
            self.path_velocities_y = [0.0] * n
        if len(self.path_rotations) < n:
            self.path_rotations = [0.0] * n

        # Add init wait phase
        init_wait = 1.0
        self.timeline.append({
            'end_time': init_wait,
            'name': 'INIT_WAIT',
            'target_vx': 0.0,
            'target_vy': 0.0,
            'target_omega': 0.0,
            'is_stopped': True
        })

        # Process each phase from the path arrays
        for i in range(n - 1):
            t_start = self.path_times[i] + init_wait
            t_end = self.path_times[i + 1] + init_wait

            vx_target = self.path_velocities_x[i]
            vy_target = self.path_velocities_y[i] if i < len(self.path_velocities_y) else 0.0
            omega_target = self.path_rotations[i] if i < len(self.path_rotations) else 0.0

            # Determine if this is a stopped phase
            is_stopped = (abs(vx_target) < 0.001 and
                         abs(vy_target) < 0.001 and
                         abs(omega_target) < 0.001)

            self.timeline.append({
                'end_time': t_end,
                'name': f'PHASE_{i+1}',
                'target_vx': vx_target,
                'target_vy': vy_target,
                'target_omega': omega_target,
                'is_stopped': is_stopped
            })

        # Add final phase (stays at last velocity forever or until cycle)
        if n > 0:
            last_vx = self.path_velocities_x[-1]
            last_vy = self.path_velocities_y[-1] if len(self.path_velocities_y) >= n else 0.0
            last_omega = self.path_rotations[-1] if len(self.path_rotations) >= n else 0.0
            is_stopped = (abs(last_vx) < 0.001 and abs(last_vy) < 0.001 and abs(last_omega) < 0.001)

            self.timeline.append({
                'end_time': float('inf'),
                'name': 'FINAL',
                'target_vx': last_vx,
                'target_vy': last_vy,
                'target_omega': last_omega,
                'is_stopped': is_stopped
            })

        self.total_duration = self.path_times[-1] + init_wait if n > 0 else init_wait

    def _build_simple_timeline(self):
        """Build timeline for simple forward-rotate-forward motion"""
        self.timeline = []

        v = self.forward_velocity
        t_trans = self.transition_time
        t_fwd = self.forward_duration
        t_rot = self.rotation_duration

        # Calculate rotation omega to achieve exact rotation_angle
        effective_rotation_time = t_rot - t_trans
        omega = self.rotation_angle / effective_rotation_time if effective_rotation_time > 0 else 0

        t = 0.0

        # Phase 0: Init wait
        t_end = 1.0
        self.timeline.append({
            'end_time': t_end,
            'name': 'INIT_WAIT',
            'target_vx': 0.0, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': True
        })
        t = t_end

        # Phase 1: Drive forward
        t_end = t + t_fwd
        self.timeline.append({
            'end_time': t_end,
            'name': 'DRIVE_1',
            'target_vx': v, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': False
        })
        t = t_end

        # Phase 2: Stop before rotation
        t_end = t + 0.5
        self.timeline.append({
            'end_time': t_end,
            'name': 'STOP_1',
            'target_vx': 0.0, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': True
        })
        t = t_end

        # Phase 3: Rotate
        t_end = t + t_rot
        self.timeline.append({
            'end_time': t_end,
            'name': 'ROTATE',
            'target_vx': 0.0, 'target_vy': 0.0, 'target_omega': omega,
            'is_stopped': False
        })
        t = t_end

        # Phase 4: Stop after rotation
        t_end = t + 0.5
        self.timeline.append({
            'end_time': t_end,
            'name': 'STOP_2',
            'target_vx': 0.0, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': True
        })
        t = t_end

        # Phase 5: Drive forward again
        t_end = t + t_fwd
        self.timeline.append({
            'end_time': t_end,
            'name': 'DRIVE_2',
            'target_vx': v, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': False
        })
        t = t_end

        # Phase 6: Final stop
        self.timeline.append({
            'end_time': float('inf'),
            'name': 'DONE',
            'target_vx': 0.0, 'target_vy': 0.0, 'target_omega': 0.0,
            'is_stopped': True
        })

        self.total_duration = t

    def _log_timeline(self):
        """Log the motion timeline"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Motion Timeline:')

        t_prev = 0.0
        for phase in self.timeline:
            t_end = phase['end_time']
            name = phase['name']
            vx = phase['target_vx']
            vy = phase['target_vy']
            omega = phase['target_omega']

            if t_end == float('inf'):
                self.get_logger().info(f'  {t_prev:.1f}s - inf: {name} (vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f})')
            else:
                self.get_logger().info(f'  {t_prev:.1f}s - {t_end:.1f}s: {name} (vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f})')
            t_prev = t_end
            if t_end == float('inf'):
                break

        self.get_logger().info('=' * 50)

    def send_initial_reset(self):
        """Send initial position reset"""
        reset_msg = PositionData()
        reset_msg.x = self.initial_position_x
        reset_msg.y = self.initial_position_y
        reset_msg.z = 0.0
        reset_msg.alpha = self.initial_alpha
        reset_msg.initial_vx = 0.0
        reset_msg.initial_vy = 0.0
        self.reset_pub.publish(reset_msg)
        self.get_logger().info(f'Initial reset: ({self.initial_position_x}, {self.initial_position_y})')
        self.initial_reset_timer.cancel()
        self.destroy_timer(self.initial_reset_timer)

    def get_phase_at_time(self, t: float) -> dict:
        """Get the phase configuration at time t"""
        for phase in self.timeline:
            if t < phase['end_time']:
                return phase
        return self.timeline[-1]

    def compute_acceleration(self, target_vx: float, target_vy: float,
                            target_omega: float) -> Tuple[float, float, float]:
        """
        Compute acceleration needed to reach target velocity from current velocity.
        Uses constant acceleration over transition_time.
        """
        t_trans = self.transition_time

        # Calculate required accelerations
        if t_trans > 0:
            ax = (target_vx - self.current_vx) / t_trans
            ay = (target_vy - self.current_vy) / t_trans
        else:
            ax = 0.0
            ay = 0.0

        # Limit accelerations to reasonable values
        max_accel = 2.0  # m/s²
        ax = max(-max_accel, min(max_accel, ax))
        ay = max(-max_accel, min(max_accel, ay))

        # Angular velocity is set directly (gyro measures angular velocity, not acceleration)
        omega = target_omega

        return ax, ay, omega

    def update_current_velocity(self, ax: float, ay: float, target_vx: float,
                                target_vy: float, target_omega: float):
        """Update current velocity based on acceleration"""
        # Integrate acceleration
        self.current_vx += ax * self.dt
        self.current_vy += ay * self.dt
        self.current_omega = target_omega

        # Clamp to target when close (prevents overshoot)
        if abs(ax) < 0.01:
            self.current_vx = target_vx
        if abs(ay) < 0.01:
            self.current_vy = target_vy

    def publish_joint_state(self):
        """Publish joint states for wheel visualization"""
        vx = self.current_vx
        vy = self.current_vy
        omega = self.current_omega

        lx = self.wheel_base_x / 2.0
        ly = self.wheel_base_y / 2.0
        k = lx + ly

        # Mecanum wheel velocities
        w1 = (vx - vy - k * omega) / self.wheel_radius
        w2 = (vx + vy + k * omega) / self.wheel_radius
        w3 = (vx - vy + k * omega) / self.wheel_radius
        w4 = (vx + vy - k * omega) / self.wheel_radius

        for i, w in enumerate([w1, w2, w3, w4]):
            self.wheel_angles[i] += w * self.dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rr_joint", "wheel_rl_joint"]
        js.position = list(self.wheel_angles)
        js.velocity = [w1, w2, w3, w4]
        self.joint_state_pub.publish(js)

    def publish_imu_data(self):
        """Publish IMU data"""
        # Handle cycling
        cycle_time = self.sim_time % self.cycle_time if self.cycle_time > 0 else self.sim_time

        # Get current phase
        phase = self.get_phase_at_time(cycle_time)
        target_vx = phase['target_vx']
        target_vy = phase['target_vy']
        target_omega = phase['target_omega']
        is_stopped = phase['is_stopped']

        # Compute acceleration to reach target velocity
        ax, ay, omega = self.compute_acceleration(target_vx, target_vy, target_omega)

        # Update internal velocity tracking
        self.update_current_velocity(ax, ay, target_vx, target_vy, target_omega)

        # Output behavior depends on motion state
        if is_stopped:
            # During stopped phases, output EXACTLY zero for ZUPT
            ax_out = 0.0
            ay_out = 0.0
            omega_out = 0.0
        else:
            ax_out = ax
            ay_out = ay
            # Add small alternating omega indicator to prevent false ZUPT during constant velocity
            if abs(omega) < 0.001 and (abs(target_vx) > 0.01 or abs(target_vy) > 0.01):
                omega_indicator = 0.01 * (1 if (int(self.sim_time * 100) % 2 == 0) else -1)
                omega_out = omega_indicator
            else:
                omega_out = omega

        # Get timestamp
        current_time = self.get_clock().now().to_msg()

        # Publish IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time
        imu_msg.header.frame_id = 'base_link'
        imu_msg.linear_acceleration.x = ax_out
        imu_msg.linear_acceleration.y = ay_out
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = omega_out
        imu_msg.orientation_covariance[0] = -1.0
        self.imu_pub.publish(imu_msg)

        # Publish AccelerationData
        accel_msg = AccelerationData()
        accel_msg.header.stamp = current_time
        accel_msg.header.frame_id = 'base_link'
        accel_msg.linear_x = ax_out
        accel_msg.linear_y = ay_out
        accel_msg.linear_z = 0.0
        accel_msg.angular_z = omega_out
        self.accel_pub.publish(accel_msg)

        # Publish joint states
        self.publish_joint_state()

        # Log every 2 seconds
        if int(self.sim_time * 10) % 20 == 0 and self.sim_time > 0:
            self.get_logger().info(
                f'[t={cycle_time:.1f}s] {phase["name"]}: '
                f'a=({ax_out:.3f}, {ay_out:.3f}) | ω={omega_out:.3f} | '
                f'v=({self.current_vx:.2f}, {self.current_vy:.2f})'
            )

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
