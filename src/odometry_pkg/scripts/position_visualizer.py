#!/usr/bin/env python3
"""
Position Visualizer Node for RVIZ2

Subscribes to PositionData and publishes visualization messages:
- nav_msgs/Path: Trajectory of the robot
- geometry_msgs/PoseStamped: Current pose
- visualization_msgs/Marker: Robot mesh marker (mecanum robot STL)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from odometry_interfaces_pkg.msg import PositionData
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
import math


class PositionVisualizer(Node):
    def __init__(self):
        super().__init__('position_visualizer')
        
        # Declare parameters for mesh
        self.declare_parameter('use_mesh', True)
        self.declare_parameter('mesh_scale', 0.001)  # STL is in mm, convert to meters
        self.use_mesh = self.get_parameter('use_mesh').value
        self.mesh_scale = self.get_parameter('mesh_scale').value

        # Geometry matching the URDF (box chassis + four wheel meshes)
        self.base_size = (0.40, 0.20, 0.10)
        self.base_offset_z = 0.05  # URDF base_link visual origin
        self.wheel_mesh = 'package://odometry_pkg/meshes/wheel.stl'
        self.wheel_frames = ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
        # Per-wheel orientation and scale: mirror right-side wheels using yaw flip + negative X scale
        self.wheel_mesh_orientation = self.rpy_to_quaternion(1.5708, 0.0, 0.0)
        yaw_flip = self.rpy_to_quaternion(1.5708, 0.0, 6.2832)
        self.wheel_orientations = {
            "wheel_fl": self.wheel_mesh_orientation,
            "wheel_fr": yaw_flip,
            "wheel_rl": yaw_flip,
            "wheel_rr": self.wheel_mesh_orientation,
        }
        self.wheel_scales = {
            "wheel_fl": (self.mesh_scale, self.mesh_scale, self.mesh_scale),
            "wheel_fr": (-self.mesh_scale, self.mesh_scale, self.mesh_scale),
            "wheel_rl": (-self.mesh_scale, self.mesh_scale, self.mesh_scale),
            "wheel_rr": (self.mesh_scale, self.mesh_scale, self.mesh_scale),
        }
        # Offsets: flip side gets +X (red), normal side gets -X; Y/Z shared
        self.wheel_offset_y = self.declare_parameter('wheel_offset_y', 0.025).value
        self.wheel_offset_z = self.declare_parameter('wheel_offset_z', -0.1).value
        self.wheel_offset_map_x = {
            "wheel_fl": -0.1,
            "wheel_fr": 0.1,
            "wheel_rl": 0.1,
            "wheel_rr": -0.1,
        }
        
        # Subscriber
        self.subscription = self.create_subscription(
            PositionData,
            '/odometry/position_from_accel',  # Will be remapped to /odometry/position_from_wheels
            self.position_callback,
            10
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/odometry/path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/odometry/pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/odometry/robot_marker', 10)
        
        # TF broadcaster for robot pose
        self.tf_broadcaster = TransformBroadcaster(self)

        # Track current pose so we can publish TF even before new data arrives
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_alpha = 0.0
        self.have_position = False

        # Keep TF alive even when no new PositionData is published
        self.tf_timer = self.create_timer(0.05, self.publish_last_tf)
        
        # Path message (accumulates poses)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        # Track last position for reset detection
        self.last_x = None
        self.last_y = None
        
        self.get_logger().info('Position Visualizer started')
        self.get_logger().info(f'Using mesh: {self.use_mesh}, scale: {self.mesh_scale}')
        self.get_logger().info('Subscribing to: /odometry/position_from_accel (remapped)')
        self.get_logger().info('Publishing: /odometry/path, /odometry/pose, /odometry/robot_marker')
        # Publish once immediately so all wheels appear without waiting for the first timer tick
        self.publish_last_tf()
    
    def position_callback(self, msg: PositionData):
        current_time = self.get_clock().now().to_msg()
        
        # Detect position reset (large jump back to origin)
        if self.last_x is not None and self.last_y is not None:
            dx = abs(msg.x - self.last_x)
            dy = abs(msg.y - self.last_y)
            # If position jumped more than 0.5m and is near origin, clear path
            if (dx > 0.5 or dy > 0.5) and abs(msg.x) < 0.1 and abs(msg.y) < 0.1:
                self.get_logger().info('Position reset detected - clearing path')
                self.path_msg.poses.clear()
        
        self.last_x = msg.x
        self.last_y = msg.y
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_alpha = msg.alpha
        self.have_position = True
        
        # Create PoseStamped
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = 'map'
        pose.pose.position.x = msg.x
        pose.pose.position.y = msg.y
        pose.pose.position.z = 0.0
        
        # Convert alpha (yaw) to quaternion
        yaw = msg.alpha
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Publish current pose
        self.pose_pub.publish(pose)
        
        # Broadcast TF transform for the robot
        self.broadcast_tf(msg, current_time)
        
        # Add to path and publish
        self.path_msg.header.stamp = current_time
        self.path_msg.poses.append(pose)
        
        # Limit path length to prevent memory issues
        max_path_length = 5000
        if len(self.path_msg.poses) > max_path_length:
            self.path_msg.poses = self.path_msg.poses[-max_path_length:]
        
        self.path_pub.publish(self.path_msg)
        
        # Create robot marker (URDF-matching multi-part mesh or arrow)
        self.publish_robot_marker(msg, current_time)

    def publish_last_tf(self):
        """Publish the most recent transform (or a zeroed one on startup)."""
        stamp = self.get_clock().now().to_msg()
        msg = PositionData()
        msg.x = self.current_x
        msg.y = self.current_y
        msg.alpha = self.current_alpha
        # Always broadcast so RVIZ has a valid frame; before data arrives this sits at origin
        self.broadcast_tf(msg, stamp)
        # Keep markers alive and ensure they spawn immediately
        self.publish_robot_marker(msg, stamp)

    def broadcast_tf(self, msg: PositionData, stamp):
        """Broadcast TF transform from map to base_link."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        
        # Quaternion from yaw
        yaw = msg.alpha
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_robot_marker(self, msg: PositionData, stamp):
        """Publish robot visualization marker."""
        if self.use_mesh:
            # Base cube matching URDF chassis (frame = base_link so TF controls pose)
            base_marker = Marker()
            base_marker.header.stamp = stamp
            base_marker.header.frame_id = 'base_link'
            base_marker.ns = 'robot'
            base_marker.id = 0
            base_marker.type = Marker.CUBE
            base_marker.action = Marker.ADD
            base_marker.frame_locked = True  # follow TF updates without re-publishing
            # Lifetime 0 means RViz keeps latest until overwritten; reusing IDs avoids churn
            base_marker.pose.position.x = 0.0
            base_marker.pose.position.y = 0.0
            base_marker.pose.position.z = self.base_offset_z
            base_marker.pose.orientation.x, base_marker.pose.orientation.y, \
                base_marker.pose.orientation.z, base_marker.pose.orientation.w = self.rpy_to_quaternion(0.0, 0.0, 0.0)
            base_marker.scale.x, base_marker.scale.y, base_marker.scale.z = self.base_size
            base_marker.color.r = 1.0
            base_marker.color.g = 0.0
            base_marker.color.b = 0.0
            base_marker.color.a = 0.7
            self.marker_pub.publish(base_marker)

            # Wheels as individual mesh markers parented to wheel TF frames (pose & rotation come directly from TF)
            for idx, frame in enumerate(self.wheel_frames, start=1):
                wheel_marker = Marker()
                wheel_marker.header.stamp = stamp
                wheel_marker.header.frame_id = frame
                wheel_marker.ns = 'robot'
                wheel_marker.id = idx
                wheel_marker.type = Marker.MESH_RESOURCE
                wheel_marker.action = Marker.ADD
                wheel_marker.frame_locked = True  # bind to wheel link TF
                # Bind exactly to the wheel link frame (position/orientation from TF)
                wheel_marker.pose.position.x = self.wheel_offset_map_x.get(frame, -0.1)
                wheel_marker.pose.position.y = self.wheel_offset_y
                wheel_marker.pose.position.z = self.wheel_offset_z
                ori = self.wheel_orientations.get(frame, self.wheel_mesh_orientation)
                wheel_marker.pose.orientation.x = ori[0]
                wheel_marker.pose.orientation.y = ori[1]
                wheel_marker.pose.orientation.z = ori[2]
                wheel_marker.pose.orientation.w = ori[3]

                sx, sy, sz = self.wheel_scales.get(frame, (self.mesh_scale, self.mesh_scale, self.mesh_scale))
                wheel_marker.scale.x = sx
                wheel_marker.scale.y = sy
                wheel_marker.scale.z = sz
                wheel_marker.mesh_resource = self.wheel_mesh
                wheel_marker.mesh_use_embedded_materials = False
                wheel_marker.color.r = 0.1
                wheel_marker.color.g = 0.1
                wheel_marker.color.b = 0.1
                wheel_marker.color.a = 0.8
                self.marker_pub.publish(wheel_marker)
        else:
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = 'map'
            marker.ns = 'robot'
            marker.id = 0
            marker.action = Marker.ADD
            marker.type = Marker.ARROW
            
            yaw = msg.alpha
            marker.pose.position.x = msg.x
            marker.pose.position.y = msg.y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Arrow size
            marker.scale.x = 0.3  # Length
            marker.scale.y = 0.05  # Width
            marker.scale.z = 0.05  # Height
            
            # Red color
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.marker_pub.publish(marker)

    @staticmethod
    def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
        """Convert roll, pitch, yaw to quaternion (x, y, z, w)."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy
        return x, y, z, w

    @staticmethod
    def multiply_quaternion(q1, q2):
        """Quaternion multiplication q = q1 * q2 (xyzw)."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return x, y, z, w


def main(args=None):
    rclpy.init(args=args)
    node = PositionVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
