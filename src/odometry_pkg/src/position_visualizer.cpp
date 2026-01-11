/**
 * @file position_visualizer.cpp
 * @brief Position Visualizer - Publishes TF and visualization markers for RViz
 *
 * This node subscribes to odometry position data and publishes:
 * - TF transforms (map -> base_link)
 * - RViz visualization markers (robot mesh + path)
 * - Path message for trajectory visualization
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;

/**
 * @class PositionVisualizer
 * @brief ROS2 node that visualizes odometry data in RViz
 */
class PositionVisualizer : public rclcpp::Node
{
public:
    PositionVisualizer() : Node("position_visualizer")
    {
        // Declare parameters
        this->declare_parameter<bool>("use_mesh", true);
        this->declare_parameter<double>("mesh_scale", 0.001);
        this->declare_parameter<std::string>("mesh_path", "");
        this->declare_parameter<double>("path_width", 0.02);
        this->declare_parameter<int>("max_path_points", 10000);

        // Get parameters
        use_mesh_ = this->get_parameter("use_mesh").as_bool();
        mesh_scale_ = this->get_parameter("mesh_scale").as_double();
        mesh_path_ = this->get_parameter("mesh_path").as_string();
        path_width_ = this->get_parameter("path_width").as_double();
        max_path_points_ = this->get_parameter("max_path_points").as_int();

        // Default mesh path
        if (mesh_path_.empty()) {
            try {
                std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
                mesh_path_ = "file://" + pkg_share + "/meshes/mecanum_robot.stl";
            } catch (...) {
                mesh_path_ = "";
            }
        }

        // Create TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Create publishers
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/odometry/path", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/odometry/pose", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/odometry/robot_marker", 10);
        label_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/odometry/position_labels", 10);

        // Create subscriber for position
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/position_from_accel",
            10,
            std::bind(&PositionVisualizer::position_callback, this, std::placeholders::_1));

        // Initialize path message
        path_msg_.header.frame_id = "map";

    // Start time will be initialized from the first incoming message's stamp
    start_time_initialized_ = false;

        RCLCPP_INFO(this->get_logger(), "Position Visualizer started");
        RCLCPP_INFO(this->get_logger(), "  Use mesh: %s", use_mesh_ ? "true" : "false");
        if (use_mesh_) {
            RCLCPP_INFO(this->get_logger(), "  Mesh path: %s", mesh_path_.c_str());
        }
    }

private:
    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get position and orientation
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        // Get yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double yaw = 2.0 * std::atan2(qz, qw);

        // Skip NaN values (used for velocity-only updates)
        if (std::isnan(x) || std::isnan(y)) {
            return;
        }

        auto stamp = msg->header.stamp;

        // Publish TF: map -> base_link
        publish_transform(x, y, z, qz, qw, stamp);

        // Publish pose
        publish_pose(x, y, z, qz, qw, stamp);

        // Add to path
        add_to_path(x, y, z, qz, qw, stamp);

        // Publish robot marker
        publish_robot_marker(x, y, z, qz, qw, stamp);

        // Publish periodic labels (every second)
        // Check if 1 second has passed since last label
        // Calculate elapsed time since the first incoming message
        rclcpp::Time stamp_time(stamp);
        if (!start_time_initialized_) {
            start_time_ = stamp_time;
            start_time_initialized_ = true;
        }
        double elapsed_time = (stamp_time - start_time_).seconds();
        
        if (elapsed_time - last_label_time_ >= 1.0) {
            publish_time_label(x, y, z, elapsed_time, stamp_time);
            last_label_time_ = elapsed_time;
        }
    }

    void publish_transform(double x, double y, double z, 
                          double qz, double qw, 
                          const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = qz;
        t.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(t);
    }

    void publish_pose(double x, double y, double z, 
                     double qz, double qw,
                     const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        pose_pub_->publish(pose);
    }

    void add_to_path(double x, double y, double z, 
                    double qz, double qw,
                    const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "map";
        
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = qz;
        pose.pose.orientation.w = qw;

        path_msg_.poses.push_back(pose);
        
        // Limit path length
        if (static_cast<int>(path_msg_.poses.size()) > max_path_points_) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        path_msg_.header.stamp = stamp;
        path_pub_->publish(path_msg_);
    }

    // Helper to convert RPY to Quaternion
    struct Quaternion {
        double x, y, z, w;
    };

    Quaternion rpy_to_quaternion(double roll, double pitch, double yaw)
    {
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);

        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }

    void publish_robot_marker(double x, double y, double z, 
                             double qz, double qw,
                             const rclcpp::Time& stamp)
    {
        // 1. Base Chassis Marker (Cube)
        visualization_msgs::msg::Marker base_marker;
        base_marker.header.stamp = stamp;
        base_marker.header.frame_id = "base_link"; 
        base_marker.ns = "robot_base";
        base_marker.id = 0;
        base_marker.type = visualization_msgs::msg::Marker::CUBE;
        base_marker.action = visualization_msgs::msg::Marker::ADD;
        base_marker.frame_locked = true;

        base_marker.scale.x = 0.40;
        base_marker.scale.y = 0.20;
        base_marker.scale.z = 0.10;

        base_marker.pose.position.x = 0.0;
        base_marker.pose.position.y = 0.0;
        base_marker.pose.position.z = 0.05;
        base_marker.pose.orientation.w = 1.0;

        base_marker.color.r = 0.1f;
        base_marker.color.g = 0.1f;
        base_marker.color.b = 0.1f;
        base_marker.color.a = 1.0f;

        marker_pub_->publish(base_marker);

        // 2. Wheels
        if (use_mesh_) {
            std::string wheel_pkg_share;
            try {
                wheel_pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
            } catch (...) {
                return; 
            }
            std::string wheel_mesh_resource = "file://" + wheel_pkg_share + "/meshes/wheel.stl";

            // Calculate orientations correct matching Python
            // Base orientation: Roll = 90 deg (1.5708 rad)
            Quaternion q_left = rpy_to_quaternion(1.5708, 0.0, 0.0);
            
            // Right orientation: Roll = 90 deg, Yaw = 180 deg (flip)
            // Note: In Python, yaw was 6.2832 (2*PI) which is same as 0, but for flip we need PI
            // Actually Python used: rpy_to_quaternion(1.5708, 0.0, 6.2832) -> effectively 0 yaw
            // But right wheels were mirrored by negative scale X!
            // Let's use negative scale X as in Python and keep same rotation for "flip" side if that's what Python did.
            // Python:
            // wheel_mesh_orientation = rpy_to_quaternion(1.5708, 0.0, 0.0)
            // yaw_flip = rpy_to_quaternion(1.5708, 0.0, 6.2832) -> this is effectively same as above (2*PI = 0)
            // So simulation used same orientation but negative scale! 
            
            Quaternion q_right = rpy_to_quaternion(1.5708, 0.0, 3.14159); // Try proper 180 flip

            struct WheelConfig {
                std::string frame_id; 
                int id;
                double x_offset;
                double scale_x;
                bool is_right;
            };

            std::vector<WheelConfig> wheels = {
                {"wheel_fl", 1, -0.1,  mesh_scale_, false},  // Front left
                {"wheel_fr", 2,  0.1, -mesh_scale_, true},   // Front right (mirrored)
                {"wheel_rl", 3,  0.1, -mesh_scale_, true},   // Rear left (mirrored)
                {"wheel_rr", 4, -0.1,  mesh_scale_, false}   // Rear right
            };

            double wheel_offset_y = 0.025;
            double wheel_offset_z = -0.1;

            for (const auto& w : wheels) {
                visualization_msgs::msg::Marker wheel_marker;
                wheel_marker.header.stamp = stamp;
                wheel_marker.header.frame_id = w.frame_id;
                wheel_marker.ns = "robot_wheels";
                wheel_marker.id = w.id;
                wheel_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
                wheel_marker.action = visualization_msgs::msg::Marker::ADD;
                wheel_marker.frame_locked = true;
                wheel_marker.mesh_resource = wheel_mesh_resource;
                
                wheel_marker.pose.position.x = w.x_offset;
                wheel_marker.pose.position.y = wheel_offset_y;
                wheel_marker.pose.position.z = wheel_offset_z;

                // Python logic used 2*PI which is 0. So actually same orientation for all?
                // "yaw_flip = self.rpy_to_quaternion(1.5708, 0.0, 6.2832)" -> 6.28 is 360 deg.
                // So python used same orientation for all, just negative scale for right.
                wheel_marker.pose.orientation.x = q_left.x;
                wheel_marker.pose.orientation.y = q_left.y;
                wheel_marker.pose.orientation.z = q_left.z;
                wheel_marker.pose.orientation.w = q_left.w;

                wheel_marker.scale.x = w.scale_x;
                wheel_marker.scale.y = mesh_scale_;
                wheel_marker.scale.z = mesh_scale_;

                wheel_marker.color.r = 0.3f;
                wheel_marker.color.g = 0.3f;
                wheel_marker.color.b = 0.3f;
                wheel_marker.color.a = 1.0f;

                marker_pub_->publish(wheel_marker);
            }
        }
    }

    void publish_time_label(double x, double y, double z, double time_sec, const rclcpp::Time & stamp)
    {
        visualization_msgs::msg::MarkerArray markers;
        
        // Sphere marker
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "map";
        sphere.header.stamp = stamp;
        sphere.ns = "position_spheres";
        sphere.id = label_id_++;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = x;
        sphere.pose.position.y = y;
        sphere.pose.position.z = z;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.1;
        sphere.scale.y = 0.1;
        sphere.scale.z = 0.1;
        sphere.color.r = 1.0; // Red
        sphere.color.g = 0.0;
        sphere.color.b = 0.0;
        sphere.color.a = 1.0;
        markers.markers.push_back(sphere);

        // Text label
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = stamp;
        text.ns = "position_labels";
        text.id = label_id_++;
        text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::msg::Marker::ADD;
        text.pose.position.x = x;
        text.pose.position.y = y;
        text.pose.position.z = z + 0.15; // Above sphere
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.1; // Text height
        text.color.r = 1.0;
        text.color.g = 1.0;
        text.color.b = 1.0;
        text.color.a = 1.0;
        
        std::stringstream ss;
        ss << "t=" << std::fixed << std::setprecision(1) << time_sec << "s";
        text.text = ss.str();
        markers.markers.push_back(text);

        label_pub_->publish(markers);
    }

    // Parameters
    bool use_mesh_;
    double mesh_scale_;
    std::string mesh_path_;
    double path_width_;
    int max_path_points_;

    // Path storage
    nav_msgs::msg::Path path_msg_;
    
    // Label tracking
    double last_label_time_{0.0};
    int label_id_{0};
    rclcpp::Time start_time_;              // Start time (initialized from first message)
    bool start_time_initialized_{false};  // True after first incoming message stamp is stored

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr label_pub_;

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
