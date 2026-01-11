/**
 * @file mecanum_simulator.cpp
 * @brief Mecanum Simulator - Reads motion paths from YAML and publishes wheel velocity data
 *
 * This simulator loads path configurations from YAML files and publishes
 * mecanum wheel velocity data.
 *
 * Supports three interpolation types:
 * - constant: Fixed value over interval
 * - linear: v(t) = m * t_rel + b
 * - parabolic: v(t) = a * t_rel² + b * t_rel + c
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// Simple YAML-like parser for path files
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

/**
 * @struct Segment
 * @brief Represents a single path segment with velocity data
 */
struct Segment {
    double start_time;
    double end_time;
    std::string seg_type;  // "constant", "linear", "parabolic"
    
    // For constant velocity
    double velocity_x{0.0};
    double velocity_y{0.0};
    double omega{0.0};
    
    // For linear/parabolic: coefficients
    double vel_x_m{0.0}, vel_x_b{0.0};
    double vel_y_m{0.0}, vel_y_b{0.0};
    double omega_m{0.0}, omega_b{0.0};
};

/**
 * @class MecanumSimulator
 * @brief ROS2 node that simulates mecanum wheel sensor data from YAML path files
 */
class MecanumSimulator : public rclcpp::Node
{
public:
    MecanumSimulator() : Node("mecanum_simulator")
    {
        // Declare parameters
        this->declare_parameter<std::string>("path_file", "");
        this->declare_parameter<int>("publish_rate_hz", 50);
        this->declare_parameter<int>("interval", -1);  // Sample interval in ms (overrides publish_rate_hz)
        this->declare_parameter<int>("decimalen", 6);  // Decimal precision for rounding
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_alpha", 0.0);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("wheel_base_x", 0.30);
        this->declare_parameter<double>("wheel_base_y", 0.25);
        this->declare_parameter<bool>("loop", false);

        // Get parameters
        path_file_ = this->get_parameter("path_file").as_string();
        rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
        int interval_ms = this->get_parameter("interval").as_int();
        decimalen_ = this->get_parameter("decimalen").as_int();
        initial_x_ = this->get_parameter("initial_x").as_double();
        initial_y_ = this->get_parameter("initial_y").as_double();
        initial_alpha_ = this->get_parameter("initial_alpha").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_base_x_ = this->get_parameter("wheel_base_x").as_double();
        wheel_base_y_ = this->get_parameter("wheel_base_y").as_double();
        loop_ = this->get_parameter("loop").as_bool();
        
        // If interval is specified, override rate_hz
        if (interval_ms > 0) {
            rate_hz_ = 1000 / interval_ms;
            RCLCPP_INFO(this->get_logger(), "Using interval: %d ms -> %d Hz", interval_ms, rate_hz_);
        }
        
        // Calculate rounding factor
        rounding_factor_ = std::pow(10.0, decimalen_);

        // Load path configuration
        if (path_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No path_file specified!");
            throw std::runtime_error("path_file parameter is required");
        }

        load_yaml_path(path_file_);
        
        // Calculate dt
        dt_ = 1.0 / rate_hz_;

        // Create publishers
        wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/wheel_encoders/velocities", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        reset_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/position/corrected", 10);

        // Timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / rate_hz_)),
            std::bind(&MecanumSimulator::publish_data, this));

        // Send initial reset after startup
        initial_reset_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MecanumSimulator::send_initial_reset, this));

        RCLCPP_INFO(this->get_logger(), "Mecanum Simulator started");
        RCLCPP_INFO(this->get_logger(), "  Path file: %s", path_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Duration: %.1fs", duration_);
        RCLCPP_INFO(this->get_logger(), "  Rate: %d Hz", rate_hz_);
        RCLCPP_INFO(this->get_logger(), "  Segments: %zu", segments_.size());
    }

private:
    void load_yaml_path(const std::string& file_path)
    {
        std::string actual_path = file_path;
        
        // Handle relative paths - try to find in package share
        if (file_path[0] != '/') {
            try {
                std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
                actual_path = pkg_share + "/" + file_path;
            } catch (...) {
                // Keep original path
            }
        }

        RCLCPP_INFO(this->get_logger(), "Loading YAML path: %s", actual_path.c_str());

        try {
            YAML::Node config = YAML::LoadFile(actual_path);
            
            if (!config["path"]) {
                throw std::runtime_error("YAML file must contain a 'path' key");
            }
            
            YAML::Node path = config["path"];
            
            // Get duration
            duration_ = path["duration"] ? path["duration"].as<double>() : 30.0;
            
            // Override rate if specified
            if (path["sample_rate_hz"]) {
                rate_hz_ = path["sample_rate_hz"].as<int>();
                dt_ = 1.0 / rate_hz_;
            }
            
            // Parse segments
            if (path["segments"]) {
                for (const auto& seg_node : path["segments"]) {
                    Segment seg;
                    
                    // Get interval
                    if (seg_node["interval"]) {
                        auto interval = seg_node["interval"];
                        seg.start_time = interval[0].as<double>();
                        seg.end_time = interval[1].as<double>();
                    }
                    
                    // Get type
                    seg.seg_type = seg_node["type"] ? seg_node["type"].as<std::string>() : "constant";
                    
                    // Parse velocity values
                    if (seg_node["velocity_x"]) {
                        if (seg_node["velocity_x"].IsScalar()) {
                            seg.velocity_x = seg_node["velocity_x"].as<double>();
                        } else if (seg_node["velocity_x"].IsMap()) {
                            if (seg.seg_type == "linear") {
                                seg.vel_x_m = seg_node["velocity_x"]["m"] ? seg_node["velocity_x"]["m"].as<double>() : 0.0;
                                seg.vel_x_b = seg_node["velocity_x"]["b"] ? seg_node["velocity_x"]["b"].as<double>() : 0.0;
                            }
                        }
                    }
                    
                    if (seg_node["velocity_y"]) {
                        if (seg_node["velocity_y"].IsScalar()) {
                            seg.velocity_y = seg_node["velocity_y"].as<double>();
                        } else if (seg_node["velocity_y"].IsMap()) {
                            if (seg.seg_type == "linear") {
                                seg.vel_y_m = seg_node["velocity_y"]["m"] ? seg_node["velocity_y"]["m"].as<double>() : 0.0;
                                seg.vel_y_b = seg_node["velocity_y"]["b"] ? seg_node["velocity_y"]["b"].as<double>() : 0.0;
                            }
                        }
                    }
                    
                    if (seg_node["omega"]) {
                        if (seg_node["omega"].IsScalar()) {
                            seg.omega = seg_node["omega"].as<double>();
                        } else if (seg_node["omega"].IsMap()) {
                            if (seg.seg_type == "linear") {
                                seg.omega_m = seg_node["omega"]["m"] ? seg_node["omega"]["m"].as<double>() : 0.0;
                                seg.omega_b = seg_node["omega"]["b"] ? seg_node["omega"]["b"].as<double>() : 0.0;
                            }
                        }
                    }
                    
                    segments_.push_back(seg);
                }
            }
            
            // Sort by start time
            std::sort(segments_.begin(), segments_.end(), 
                      [](const Segment& a, const Segment& b) { return a.start_time < b.start_time; });
                      
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parse error: %s", e.what());
            throw;
        }
    }

    void send_initial_reset()
    {
        // Cancel one-shot timer
        initial_reset_timer_->cancel();
        
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        msg.pose.pose.position.x = initial_x_;
        msg.pose.pose.position.y = initial_y_;
        msg.pose.pose.position.z = 0.0;
        msg.pose.pose.orientation.z = std::sin(initial_alpha_ / 2.0);
        msg.pose.pose.orientation.w = std::cos(initial_alpha_ / 2.0);
        msg.twist.twist.linear.x = 0.0;
        msg.twist.twist.linear.y = 0.0;
        
        reset_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Initial reset: (%.2f, %.2f, α=%.2f)", 
                    initial_x_, initial_y_, initial_alpha_);
    }

    const Segment* get_current_segment(double t)
    {
        for (const auto& seg : segments_) {
            if (t >= seg.start_time && t < seg.end_time) {
                return &seg;
            }
        }
        // If past all segments, return the last one
        if (!segments_.empty() && t >= segments_.back().end_time) {
            return &segments_.back();
        }
        return nullptr;
    }

    double evaluate_velocity(const Segment& seg, double t_rel, 
                             double constant_val, double m, double b)
    {
        if (seg.seg_type == "linear") {
            return m * t_rel + b;
        }
        return constant_val;
    }

    void get_velocity_at_time(double t, double& vx, double& vy, double& omega)
    {
        const Segment* seg = get_current_segment(t);
        
        if (!seg) {
            vx = vy = omega = 0.0;
            return;
        }

        double t_rel = t - seg->start_time;

        vx = evaluate_velocity(*seg, t_rel, seg->velocity_x, seg->vel_x_m, seg->vel_x_b);
        vy = evaluate_velocity(*seg, t_rel, seg->velocity_y, seg->vel_y_m, seg->vel_y_b);
        omega = evaluate_velocity(*seg, t_rel, seg->omega, seg->omega_m, seg->omega_b);
    }

    void publish_data()
    {
        // Handle looping
        if (loop_ && sim_time_ >= duration_) {
            sim_time_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Path looped - restarting from beginning");
        }

        double vx, vy, omega;
        get_velocity_at_time(sim_time_, vx, vy, omega);
        
        // Round to specified decimal places (decimalen parameter)
        vx = std::round(vx * rounding_factor_) / rounding_factor_;
        vy = std::round(vy * rounding_factor_) / rounding_factor_;
        omega = std::round(omega * rounding_factor_) / rounding_factor_;

        auto current_time = this->now();
        publish_mecanum_data(vx, vy, omega, current_time);

        // Log every 2 seconds
        static int log_counter = 0;
        if (++log_counter >= rate_hz_ * 2) {
            const Segment* seg = get_current_segment(sim_time_);
            std::string seg_name = seg ? seg->seg_type : "none";
            RCLCPP_INFO(this->get_logger(), "[t=%.1fs] %s: vx=%.3f, vy=%.3f, ω=%.3f",
                       sim_time_, seg_name.c_str(), vx, vy, omega);
            log_counter = 0;
        }

        sim_time_ += dt_;
    }

    void publish_mecanum_data(double vx, double vy, double omega, const rclcpp::Time& stamp)
    {
        // Calculate mecanum wheel velocities
        double lx = wheel_base_x_ / 2.0;
        double ly = wheel_base_y_ / 2.0;
        double k = lx + ly;
        double r = wheel_radius_;

        double w1 = (vx - vy - k * omega) / r;  // FL
        double w2 = (vx + vy + k * omega) / r;  // FR
        double w3 = (vx - vy + k * omega) / r;  // RR
        double w4 = (vx + vy - k * omega) / r;  // RL

        // Update wheel angles
        wheel_angles_[0] += w1 * dt_;
        wheel_angles_[1] += w2 * dt_;
        wheel_angles_[2] += w3 * dt_;
        wheel_angles_[3] += w4 * dt_;

        // Publish wheel velocities
        auto wheel_msg = std_msgs::msg::Float64MultiArray();
        wheel_msg.data = {w1, w2, w3, w4};
        wheel_vel_pub_->publish(wheel_msg);

        // Publish joint states for visualization
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = stamp;
        js.name = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rr_joint", "wheel_rl_joint"};
        js.position = {wheel_angles_[0], wheel_angles_[1], wheel_angles_[2], wheel_angles_[3]};
        js.velocity = {w1, w2, w3, w4};
        joint_state_pub_->publish(js);
    }

    // Member variables
    std::string path_file_;
    int rate_hz_;
    int decimalen_;
    double rounding_factor_;
    double dt_;
    double duration_{30.0};
    double initial_x_, initial_y_, initial_alpha_;
    double wheel_radius_, wheel_base_x_, wheel_base_y_;
    bool loop_;
    double sim_time_{0.0};
    std::array<double, 4> wheel_angles_{{0.0, 0.0, 0.0, 0.0}};
    
    std::vector<Segment> segments_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reset_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr initial_reset_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MecanumSimulator>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mecanum_simulator"), "Error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
