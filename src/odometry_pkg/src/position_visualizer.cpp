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
#include <array>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
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
        this->declare_parameter<int>("total_segments", 5);

        // Get parameters
        use_mesh_ = this->get_parameter("use_mesh").as_bool();
        mesh_scale_ = this->get_parameter("mesh_scale").as_double();
        mesh_path_ = this->get_parameter("mesh_path").as_string();
        path_width_ = this->get_parameter("path_width").as_double();
        max_path_points_ = this->get_parameter("max_path_points").as_int();
        total_segments_ = this->get_parameter("total_segments").as_int();

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
        status_label_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/odometry/status_label", 10);

        // Create subscriber for position
        position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/position_from_accel",
            10,
            std::bind(&PositionVisualizer::position_callback, this, std::placeholders::_1));

        // Create subscriber for segment info
        segment_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/simulator/segment_info",
            10,
            std::bind(&PositionVisualizer::segment_callback, this, std::placeholders::_1));

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

        // Update status tracking (speed, distance, accel)
        update_status_tracking(x, y, msg->twist.twist.linear.x, msg->twist.twist.linear.y, stamp);

        // Publish floating status label above robot
        publish_floating_status(x, y, z, stamp);

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

    void segment_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            current_segment_ = msg->data[0];
            total_segments_ = msg->data[1];
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

        base_marker.color.r = 0.0f;
        base_marker.color.g = 0.0f;
        base_marker.color.b = 0.5f;
        base_marker.color.a = 0.9f;

        marker_pub_->publish(base_marker);

        // Wheels are rendered via URDF RobotModel in RViz - no need for separate markers
    }

    void publish_time_label(double x, double y, double z, double /*time_sec*/, const rclcpp::Time & stamp)
    {
        visualization_msgs::msg::MarkerArray markers;
        
        // Sphere marker with segment-based color
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
        
        // Color based on current segment (different color per segment)
        set_sphere_color_by_segment(sphere);
        
        markers.markers.push_back(sphere);

        // No text labels - removed per user request 
        // (floating status label above robot shows all info now)

        label_pub_->publish(markers);
    }

    // Helper to set sphere color based on current segment
    void set_sphere_color_by_segment(visualization_msgs::msg::Marker& sphere)
    {
        // Color palette for segments (up to 10 distinct colors)
        const std::vector<std::array<float, 3>> segment_colors = {
            {1.0f, 0.0f, 0.0f},    // Segment 1: Red
            {0.0f, 1.0f, 0.0f},    // Segment 2: Green
            {0.0f, 0.5f, 1.0f},    // Segment 3: Blue
            {1.0f, 1.0f, 0.0f},    // Segment 4: Yellow
            {1.0f, 0.0f, 1.0f},    // Segment 5: Magenta
            {0.0f, 1.0f, 1.0f},    // Segment 6: Cyan
            {1.0f, 0.5f, 0.0f},    // Segment 7: Orange
            {0.5f, 0.0f, 1.0f},    // Segment 8: Purple
            {0.0f, 1.0f, 0.5f},    // Segment 9: Teal
            {1.0f, 0.0f, 0.5f}     // Segment 10: Pink
        };
        
        // Get color index (0-indexed, wrap around if more than 10 segments)
        int color_idx = (current_segment_ - 1) % segment_colors.size();
        
        sphere.color.r = segment_colors[color_idx][0];
        sphere.color.g = segment_colors[color_idx][1];
        sphere.color.b = segment_colors[color_idx][2];
        sphere.color.a = 1.0f;
    }

    void update_status_tracking(double x, double y, double vx, double vy, 
                                const builtin_interfaces::msg::Time& stamp)
    {
        rclcpp::Time current_time(stamp);
        
        if (prev_initialized_) {
            // Calculate distance increment
            double dx = x - prev_x_;
            double dy = y - prev_y_;
            double dist_increment = std::sqrt(dx * dx + dy * dy);
            total_distance_ += dist_increment;
            
            // Calculate current speed from velocity components
            current_speed_ = std::sqrt(vx * vx + vy * vy);
            
            // Calculate acceleration from velocity change
            double dt = (current_time - prev_time_).seconds();
            if (dt > 0.0) {
                double dvx = vx - prev_vx_;
                double dvy = vy - prev_vy_;
                double accel_x = dvx / dt;
                double accel_y = dvy / dt;
                current_accel_ = std::sqrt(accel_x * accel_x + accel_y * accel_y);
            }
            
            prev_vx_ = vx;
            prev_vy_ = vy;
        }
        
        prev_x_ = x;
        prev_y_ = y;
        prev_time_ = current_time;
        prev_initialized_ = true;
    }

    void publish_floating_status(double x, double y, double z, 
                                 const builtin_interfaces::msg::Time& stamp)
    {
        visualization_msgs::msg::Marker status;
        status.header.frame_id = "map";
        status.header.stamp = stamp;
        status.ns = "status_label";
        status.id = 0;
        status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        status.action = visualization_msgs::msg::Marker::ADD;
        
        // Position above the robot (higher than time labels)
        status.pose.position.x = x;
        status.pose.position.y = y;
        status.pose.position.z = z + 0.40;
        status.pose.orientation.w = 1.0;
        
        // Text size
        status.scale.z = 0.08;
        
        // White text
        status.color.r = 1.0f;
        status.color.g = 1.0f;
        status.color.b = 1.0f;
        status.color.a = 1.0f;
        
        // Calculate elapsed time
        double elapsed_time = 0.0;
        if (start_time_initialized_) {
            rclcpp::Time current_time(stamp);
            elapsed_time = (current_time - start_time_).seconds();
        }
        
        // Format the multi-line status text like in the screenshot
        std::stringstream ss;
        ss << std::fixed;
        ss << "seg:" << current_segment_ << "/" << total_segments_ << "\n";
        ss << "time:" << std::setprecision(0) << elapsed_time << "s\n";
        ss << "dist:" << std::setprecision(2) << total_distance_ << "m\n";
        ss << "speed:" << std::setprecision(2) << current_speed_ << "m/s\n";
        ss << "accel:" << std::setprecision(2) << current_accel_ << "m/s^2";
        
        status.text = ss.str();
        
        status_label_pub_->publish(status);
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

    // Status tracking for floating label
    double total_distance_{0.0};           // Total distance traveled (meters)
    double current_speed_{0.0};            // Current speed (m/s)
    double current_accel_{0.0};            // Current acceleration magnitude (m/s^2)
    double prev_x_{0.0}, prev_y_{0.0};     // Previous position for distance calculation
    double prev_vx_{0.0}, prev_vy_{0.0};   // Previous velocity for acceleration calculation
    rclcpp::Time prev_time_;               // Previous timestamp
    bool prev_initialized_{false};         // True after first position received
    int current_segment_{1};               // Current segment number
    int total_segments_{5};                // Total segments (will be updated from parameter)
    
    // Publisher for floating status label
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr status_label_pub_;

    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr label_pub_;

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr segment_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
