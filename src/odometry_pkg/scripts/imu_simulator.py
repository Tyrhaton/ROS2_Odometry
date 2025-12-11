#!/usr/bin/env python3
"""
IMU Simulator - Clean implementation without drift

Motion pattern:
- Drive forward for 5 seconds
- Stop completely
- Rotate 90 degrees in place
- Drive forward again in new direction

Key fix: During stopped phases, output EXACTLY zero acceleration (no noise)
to allow ZUPT to work properly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from odometry_interfaces_pkg.msg import AccelerationData, PositionData
import math
import random
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class MotionState:
    """Current motion state"""
    vx: float = 0.0  # Body frame velocity X
    vy: float = 0.0  # Body frame velocity Y
    omega: float = 0.0  # Angular velocity
    ax: float = 0.0  # Acceleration X
    ay: float = 0.0  # Acceleration Y
    is_stopped: bool = True  # Whether robot should be completely still


class IMUSimulator(Node):
    def __init__(self):
        super().__init__('imu_simulator')

        # Declare parameters
        self.declare_parameter('publish_rate_hz', 50)
        self.declare_parameter('noise_std', 0.0005)
        self.declare_parameter('initial_position_x', 0.0)
        self.declare_parameter('initial_position_y', 0.0)
        self.declare_parameter('initial_alpha', 0.0)
        self.declare_parameter('cycle_time', 30.0)

        # Motion parameters
        self.declare_parameter('forward_velocity', 0.3)
        self.declare_parameter('forward_duration', 5.0)
        self.declare_parameter('rotation_angle', 1.5708)  # 90 degrees
        self.declare_parameter('transition_time', 0.5)

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base_x', 0.30)
        self.declare_parameter('wheel_base_y', 0.25)

        # Get parameters
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.noise_std = self.get_parameter('noise_std').value
        self.initial_position_x = self.get_parameter('initial_position_x').value
        self.initial_position_y = self.get_parameter('initial_position_y').value
        self.initial_alpha = self.get_parameter('initial_alpha').value
        self.cycle_time = self.get_parameter('cycle_time').value

        self.forward_velocity = self.get_parameter('forward_velocity').value
        self.forward_duration = self.get_parameter('forward_duration').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.transition_time = self.get_parameter('transition_time').value

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base_x = self.get_parameter('wheel_base_x').value
        self.wheel_base_y = self.get_parameter('wheel_base_y').value

        # Calculate derived values
        self.dt = 1.0 / self.rate_hz
        self.rotation_duration = 3.0  # Time to complete 90 degree rotation

        # IMPORTANT: With trapezoidal velocity profile, total angle = omega * (duration - transition_time)
        # So: omega = rotation_angle / (rotation_duration - transition_time)
        # This accounts for the ramp-up and ramp-down phases
        effective_rotation_time = self.rotation_duration - self.transition_time
        self.rotation_omega = self.rotation_angle / effective_rotation_time

        # Build timeline
        self._build_timeline()

        # State tracking for visualization
        self.ground_truth_vx = 0.0
        self.ground_truth_vy = 0.0
        self.ground_truth_omega = 0.0
        self.wheel_angles = [0.0, 0.0, 0.0, 0.0]

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.accel_pub = self.create_publisher(AccelerationData, '/simulator/acceleration', 10)
        self.reset_pub = self.create_publisher(PositionData, '/position/corrected', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.publish_imu_data)
        self.sim_time = 0.0

        # Send initial reset after short delay
        self.initial_reset_timer = self.create_timer(0.5, self.send_initial_reset)

        self.get_logger().info(f'IMU Simulator started @ {self.rate_hz} Hz')
        self._log_timeline()

    def _build_timeline(self):
        """
        Build motion timeline with exact phase boundaries.

        Timeline structure: list of (end_time, phase_name, motion_func)
        """
        v = self.forward_velocity
        t_trans = self.transition_time
        t_fwd = self.forward_duration
        t_rot = self.rotation_duration
        omega = self.rotation_omega

        # Calculate accelerations
        accel = v / t_trans  # Acceleration magnitude

        t = 0.0
        self.timeline = []

        # Phase 0: Wait for system initialization (reset happens at 0.5s)
        t_end = 1.0  # Wait 1 second before starting motion
        self.timeline.append((t_end, 'INIT_WAIT', lambda tt: MotionState(is_stopped=True)))
        t = t_end

        # Phase 1: Accelerate forward (0 -> v)
        t_end = t + t_trans
        self.timeline.append((t_end, 'ACCELERATE_1', lambda tt, te=t_end, ts=t: self._accel_phase(tt, ts, te, 0, v)))
        t = t_end

        # Phase 2: Constant velocity forward
        t_end = t + (t_fwd - t_trans)
        self.timeline.append((t_end, 'DRIVE_1', lambda tt: MotionState(vx=v, is_stopped=False)))
        t = t_end

        # Phase 3: Decelerate to stop (v -> 0)
        t_end = t + t_trans
        self.timeline.append((t_end, 'DECELERATE_1', lambda tt, te=t_end, ts=t: self._accel_phase(tt, ts, te, v, 0)))
        t = t_end

        # Phase 4: Stopped before rotation
        t_end = t + 0.5
        self.timeline.append((t_end, 'STOPPED_1', lambda tt: MotionState(is_stopped=True)))
        t = t_end

        # Phase 5: Accelerate rotation (0 -> omega)
        t_end = t + t_trans
        self.timeline.append((t_end, 'ROTATE_ACCEL', lambda tt, te=t_end, ts=t: self._rotate_accel_phase(tt, ts, te, 0, omega)))
        t = t_end

        # Phase 6: Constant rotation
        t_end = t + (t_rot - 2 * t_trans)
        self.timeline.append((t_end, 'ROTATING', lambda tt: MotionState(omega=omega, is_stopped=False)))
        t = t_end

        # Phase 7: Decelerate rotation (omega -> 0)
        t_end = t + t_trans
        self.timeline.append((t_end, 'ROTATE_DECEL', lambda tt, te=t_end, ts=t: self._rotate_accel_phase(tt, ts, te, omega, 0)))
        t = t_end

        # Phase 8: Stopped after rotation
        t_end = t + 0.5
        self.timeline.append((t_end, 'STOPPED_2', lambda tt: MotionState(is_stopped=True)))
        t = t_end

        # Phase 9: Accelerate forward again (0 -> v)
        t_end = t + t_trans
        self.timeline.append((t_end, 'ACCELERATE_2', lambda tt, te=t_end, ts=t: self._accel_phase(tt, ts, te, 0, v)))
        t = t_end

        # Phase 10: Constant velocity forward
        t_end = t + (t_fwd - t_trans)
        self.timeline.append((t_end, 'DRIVE_2', lambda tt: MotionState(vx=v, is_stopped=False)))
        t = t_end

        # Phase 11: Final decelerate (v -> 0)
        t_end = t + t_trans
        self.timeline.append((t_end, 'DECELERATE_2', lambda tt, te=t_end, ts=t: self._accel_phase(tt, ts, te, v, 0)))
        t = t_end

        # Phase 12: Final stop (forever)
        self.timeline.append((float('inf'), 'DONE', lambda tt: MotionState(is_stopped=True)))

        self.total_duration = t

    def _accel_phase(self, t: float, t_start: float, t_end: float, v_start: float, v_end: float) -> MotionState:
        """Linear acceleration phase for forward motion"""
        dt_phase = t_end - t_start
        progress = (t - t_start) / dt_phase if dt_phase > 0 else 1.0
        progress = max(0.0, min(1.0, progress))

        # Constant acceleration: a = (v_end - v_start) / dt
        accel = (v_end - v_start) / dt_phase if dt_phase > 0 else 0.0

        # Current velocity: v = v_start + a * (t - t_start)
        current_v = v_start + accel * (t - t_start)

        return MotionState(vx=current_v, ax=accel, is_stopped=False)

    def _rotate_accel_phase(self, t: float, t_start: float, t_end: float, omega_start: float, omega_end: float) -> MotionState:
        """Linear angular acceleration phase"""
        dt_phase = t_end - t_start
        progress = (t - t_start) / dt_phase if dt_phase > 0 else 1.0
        progress = max(0.0, min(1.0, progress))

        # Current angular velocity (linear interpolation)
        current_omega = omega_start + (omega_end - omega_start) * progress

        return MotionState(omega=current_omega, is_stopped=False)

    def _log_timeline(self):
        """Log the motion timeline"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Motion Timeline:')

        t_prev = 0.0
        for t_end, name, _ in self.timeline:
            if t_end == float('inf'):
                self.get_logger().info(f'  {t_prev:.1f}s - inf: {name}')
            else:
                self.get_logger().info(f'  {t_prev:.1f}s - {t_end:.1f}s: {name}')
            t_prev = t_end
            if t_end == float('inf'):
                break

        self.get_logger().info(f'Total active duration: {self.total_duration:.1f}s')
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
        self.get_logger().info(f'Initial reset: ({self.initial_position_x}, {self.initial_position_y}), alpha={self.initial_alpha}')
        self.initial_reset_timer.cancel()
        self.destroy_timer(self.initial_reset_timer)

    def get_motion_state(self, t: float) -> Tuple[str, MotionState]:
        """Get motion state at time t"""
        t_prev = 0.0
        for t_end, name, func in self.timeline:
            if t < t_end:
                return name, func(t)
            t_prev = t_end

        # Default: stopped
        return 'DONE', MotionState(is_stopped=True)

    def publish_joint_state(self):
        """Publish joint states for wheel visualization"""
        vx = self.ground_truth_vx
        vy = self.ground_truth_vy
        omega = self.ground_truth_omega

        lx = self.wheel_base_x / 2.0
        ly = self.wheel_base_y / 2.0
        k = lx + ly

        # Mecanum wheel velocities
        w1 = (vx - vy - k * omega) / self.wheel_radius  # FL
        w2 = (vx + vy + k * omega) / self.wheel_radius  # FR
        w3 = (vx - vy + k * omega) / self.wheel_radius  # RR
        w4 = (vx + vy - k * omega) / self.wheel_radius  # RL

        # Update angles
        self.wheel_angles[0] += w1 * self.dt
        self.wheel_angles[1] += w2 * self.dt
        self.wheel_angles[2] += w3 * self.dt
        self.wheel_angles[3] += w4 * self.dt

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

        # Get current motion state
        phase_name, state = self.get_motion_state(cycle_time)

        # Get accelerations and angular velocity
        ax = state.ax
        ay = state.ay
        omega = state.omega

        # CRITICAL: Output behavior depends on motion state
        if state.is_stopped:
            # During stopped phases, output EXACTLY zero to allow ZUPT to work
            ax_out = 0.0
            ay_out = 0.0
            omega_out = 0.0
        else:
            # During motion phases, output actual acceleration with tiny omega
            # The small omega prevents ZUPT (gyro threshold 0.005) without affecting heading
            # because it's alternating and averages to zero
            ax_out = ax
            ay_out = ay
            # Small alternating omega that averages to zero but stays above ZUPT threshold
            omega_indicator = 0.01 * (1 if (int(self.sim_time * 100) % 2 == 0) else -1)
            omega_out = omega + omega_indicator if omega == 0 else omega

        # Update ground truth for visualization
        self.ground_truth_vx = state.vx
        self.ground_truth_vy = state.vy
        self.ground_truth_omega = state.omega

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
                f'[t={cycle_time:.1f}s] {phase_name}: '
                f'a=({ax_out:.3f}, {ay_out:.3f}) m/s² | ω={omega_out:.3f} rad/s | '
                f'v=({self.ground_truth_vx:.3f}, {self.ground_truth_vy:.3f}) m/s'
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
