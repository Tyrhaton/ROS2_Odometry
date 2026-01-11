/**
 * @file mecanum_simulator.cpp
 * @brief Mecanum Simulator - Lagrange Interpolation Implementation
 *
 * Reads velocity profiles from YAML files and publishes mecanum wheel data.
 * Uses Lagrange interpolation for smooth velocity profiles.
 *
 * Supports three interpolation types:
 * - constant: Fixed value over interval (scalar)
 * - linear: v(t) = m * t + b (legacy coefficient format)
 * - quadratic: v(t) = a * t² + b * t + c (legacy coefficient format)
 * - interpolation points: [[t0, v0], [t1, v1], ...] (Lagrange interpolation)
 *
 * @author Group g1 - Lagrange interpolation version
 * @date 2026-01-11
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <vector>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

/**
 * @struct InterpolationPoint
 * @brief A single data point for Lagrange interpolation
 */
struct InterpolationPoint {
    double time;    // Time value (relative to segment start)
    double value;   // Velocity value at this time
};

/**
 * @struct VelocityProfile
 * @brief Lagrange interpolation profile for one velocity axis
 * 
 * Stores a set of (time, velocity) points and performs
 * Lagrange polynomial interpolation between them.
 */
struct VelocityProfile {
    std::vector<InterpolationPoint> points;
    
    /**
     * @brief Calculate velocity at relative time using piecewise linear Lagrange interpolation
     * @param t_rel Time relative to segment start
     * @return Interpolated velocity value at this time
     */
    double evaluate(double t_rel) const {
        // Handle edge cases
        if (points.empty()) {
            return 0.0;
        }
        
        if (points.size() == 1) {
            return points[0].value;
        }
        
        // PIECEWISE LINEAR LAGRANGE INTERPOLATION
        // Find the interval [xi, xi+1] that contains t_rel
        
        // If before first point, use first value
        if (t_rel <= points[0].time) {
            return points[0].value;
        }
        
        // If after last point, use last value
        if (t_rel >= points.back().time) {
            return points.back().value;
        }
        
        // Find the right interval
        for (size_t i = 0; i < points.size() - 1; ++i) {
            if (t_rel >= points[i].time && t_rel <= points[i+1].time) {
                // LINEAR PIECEWISE LAGRANGE between points[i] and points[i+1]
                double xi = points[i].time;
                double xi1 = points[i+1].time;
                double fi = points[i].value;
                double fi1 = points[i+1].value;
                
                // Avoid division by zero
                if (std::abs(xi1 - xi) < 1e-10) {
                    return fi;
                }
                
                // Lagrange linear interpolation formula
                double L0 = (t_rel - xi1) / (xi - xi1);
                double L1 = (t_rel - xi) / (xi1 - xi);
                
                return fi * L0 + fi1 * L1;
            }
        }
        
        // Fallback (should never reach here)
        return points.back().value;
    }
};

/**
 * @struct TimeSegment
 * @brief Defines velocity behavior during a specific time interval
 */
struct TimeSegment {
    double start_time;  // When this segment begins (seconds)
    double end_time;    // When this segment ends (seconds)
    
    VelocityProfile velocity_x;  // X-axis velocity profile (m/s)
    VelocityProfile velocity_y;  // Y-axis velocity profile (m/s)
    VelocityProfile omega;       // Angular velocity profile (rad/s)
    
    /**
     * @brief Check if a given time falls within this segment
     */
    bool contains_time(double t) const {
        return (t >= start_time) && (t < end_time);
    }
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
        // PARAMETER SETUP
        declare_and_get_parameters();
        
        // LOAD PATH FROM YAML
        load_path_from_yaml();
        
        // CREATE PUBLISHERS
        wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/wheel_encoders/velocities", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);
        reset_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/position/corrected", 10);
        
        // CREATE TIMERS
        double timer_period_ms = 1000.0 / rate_hz_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period_ms)),
            std::bind(&MecanumSimulator::timer_callback, this));
        
        // One-shot timer to send initial position reset
        initial_reset_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&MecanumSimulator::send_initial_position_reset, this));
        
        // STARTUP INFO
        RCLCPP_INFO(this->get_logger(), "- MECANUM SIMULATOR STARTED");
        RCLCPP_INFO(this->get_logger(), "- Path file:  %-26s -", path_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "- Duration:   %-26.1f -", total_duration_);
        RCLCPP_INFO(this->get_logger(), "- Rate:       %-26d -", rate_hz_);
        RCLCPP_INFO(this->get_logger(), "- Segments:   %-26zu -", segments_.size());
    }

private:
    //
    // PARAMETER MANAGEMENT
    //
    
    void declare_and_get_parameters()
    {
        this->declare_parameter<std::string>("path_file", "");
        this->declare_parameter<int>("publish_rate_hz", 50);
        this->declare_parameter<int>("interval", -1);
        this->declare_parameter<int>("decimalen", 6);
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_alpha", 0.0);
        this->declare_parameter<double>("wheel_radius", 0.05);
        this->declare_parameter<double>("wheel_base_x", 0.30);
        this->declare_parameter<double>("wheel_base_y", 0.25);
        this->declare_parameter<bool>("loop", false);

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
            RCLCPP_INFO(this->get_logger(), 
                "Using interval parameter: %d ms -> %d Hz", interval_ms, rate_hz_);
        }
        
        // Calculate rounding factor based on decimalen
        rounding_factor_ = std::pow(10.0, decimalen_);
        dt_ = 1.0 / rate_hz_;
        
        if (path_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "ERROR: path_file parameter is required!");
            throw std::runtime_error("Missing required parameter: path_file");
        }
    }
    
    //
    // YAML LOADING AND PARSING
    //
    
    void load_path_from_yaml()
    {
        std::string full_path = resolve_file_path(path_file_);
        RCLCPP_INFO(this->get_logger(), "Loading YAML: %s", full_path.c_str());

        try {
            YAML::Node config = YAML::LoadFile(full_path);
            
            if (!config["path"]) {
                throw std::runtime_error("YAML must contain 'path' key");
            }
            
            YAML::Node path = config["path"];
            
            // Read total duration
            total_duration_ = path["duration"] ? path["duration"].as<double>() : 30.0;
            
            // Read sample rate from YAML (only if interval parameter was not set)
            if (path["sample_rate_hz"] && this->get_parameter("interval").as_int() <= 0) {
                rate_hz_ = path["sample_rate_hz"].as<int>();
                dt_ = 1.0 / rate_hz_;
                RCLCPP_INFO(this->get_logger(), 
                    "Using sample_rate_hz from YAML: %d Hz", rate_hz_);
            }
            
            // Parse all segments
            if (!path["segments"]) {
                throw std::runtime_error("YAML must contain 'segments' array");
            }
            
            for (const auto& seg_node : path["segments"]) {
                TimeSegment segment = parse_segment(seg_node);
                segments_.push_back(segment);
                
                RCLCPP_DEBUG(this->get_logger(), 
                    "Loaded segment [%.1f, %.1f]", 
                    segment.start_time, segment.end_time);
            }
            
            // Sort segments by start time
            std::sort(segments_.begin(), segments_.end(),
                [](const TimeSegment& a, const TimeSegment& b) {
                    return a.start_time < b.start_time;
                });
                
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
            throw;
        }
    }
    
    /**
     * @brief Convert relative path to absolute path using package share directory
     */
    std::string resolve_file_path(const std::string& file_path)
    {
        if (!file_path.empty() && file_path[0] == '/') {
            return file_path;
        }
        
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
            return pkg_share + "/" + file_path;
        } catch (...) {
            return file_path;
        }
    }
    
    /**
     * @brief Parse a single YAML segment into a TimeSegment struct
     */
    TimeSegment parse_segment(const YAML::Node& seg_node)
    {
        TimeSegment seg;
        
        // Parse time interval
        if (!seg_node["interval"]) {
            throw std::runtime_error("Segment must have 'interval' key");
        }
        auto interval = seg_node["interval"];
        seg.start_time = interval[0].as<double>();
        seg.end_time = interval[1].as<double>();
        
        double segment_duration = seg.end_time - seg.start_time;
        
        // Parse velocity profiles
        seg.velocity_x = parse_velocity_profile(seg_node, "velocity_x", segment_duration);
        seg.velocity_y = parse_velocity_profile(seg_node, "velocity_y", segment_duration);
        seg.omega = parse_velocity_profile(seg_node, "omega", segment_duration);
        
        return seg;
    }
    
    /**
     * @brief Parse velocity profile for a single axis
     * 
     * Supports multiple YAML formats:
     * - Scalar: velocity_x: 0.3 (constant)
     * - Interpolation points: velocity_x: [[0, 0], [1, 0.3], [2, 0.3]]
     * - Legacy linear: velocity_x: {m: 0.1, b: 0.0}
     * - Legacy quadratic: velocity_x: {a: -0.01, b: -0.02, c: 0.3}
     */
    VelocityProfile parse_velocity_profile(
        const YAML::Node& seg_node, 
        const std::string& key,
        double segment_duration)
    {
        VelocityProfile profile;
        
        if (!seg_node[key]) {
            // No value specified, default to zero
            profile.points.push_back({0.0, 0.0});
            return profile;
        }
        
        auto vel_node = seg_node[key];
        
        // CASE 1: Simple scalar value (constant velocity)
        if (vel_node.IsScalar()) {
            double value = vel_node.as<double>();
            // Create two points with same value for constant interpolation
            profile.points.push_back({0.0, value});
            profile.points.push_back({segment_duration, value});
            return profile;
        }
        
        // CASE 2: Sequence of [time, velocity] pairs
        if (vel_node.IsSequence()) {
            for (const auto& point_node : vel_node) {
                if (point_node.IsSequence() && point_node.size() >= 2) {
                    InterpolationPoint pt;
                    pt.time = point_node[0].as<double>();
                    pt.value = point_node[1].as<double>();
                    profile.points.push_back(pt);
                }
            }
            
            // Sort points by time
            std::sort(profile.points.begin(), profile.points.end(),
                [](const InterpolationPoint& a, const InterpolationPoint& b) {
                    return a.time < b.time;
                });
            
            if (profile.points.empty()) {
                RCLCPP_WARN(this->get_logger(), 
                    "No valid interpolation points found for '%s', using zero", 
                    key.c_str());
                profile.points.push_back({0.0, 0.0});
            }
            
            return profile;
        }
        
        // CASE 3: Legacy format (map with coefficients)
        if (vel_node.IsMap()) {
            // Linear format: v(t) = m*t + b
            if (vel_node["m"] && vel_node["b"]) {
                double m = vel_node["m"].as<double>();
                double b = vel_node["b"].as<double>();
                
                // Generate points every 1ms for ultra-smooth interpolation
                double step = 0.001;
                int num_points = std::max(2, static_cast<int>(segment_duration / step) + 1);
                
                for (int i = 0; i < num_points; ++i) {
                    double t_rel = std::min(i * step, segment_duration);
                    double vel = m * t_rel + b;
                    profile.points.push_back({t_rel, vel});
                }
                
                // Ensure last point is exactly at segment_duration
                if (profile.points.back().time < segment_duration) {
                    double vel = m * segment_duration + b;
                    profile.points.push_back({segment_duration, vel});
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Converted linear (m=%.3f, b=%.3f) to %zu points for %s",
                    m, b, profile.points.size(), key.c_str());
                return profile;
            }
            
            // Quadratic format: v(t) = a*t² + b*t + c
            if (vel_node["a"] || vel_node["b"] || vel_node["c"]) {
                double a = vel_node["a"] ? vel_node["a"].as<double>() : 0.0;
                double b = vel_node["b"] ? vel_node["b"].as<double>() : 0.0;
                double c = vel_node["c"] ? vel_node["c"].as<double>() : 0.0;
                
                // Generate points every 1ms for ultra-smooth interpolation
                double step = 0.001;
                int num_points = std::max(3, static_cast<int>(segment_duration / step) + 1);
                
                for (int i = 0; i < num_points; ++i) {
                    double t_rel = std::min(i * step, segment_duration);
                    double vel = a * t_rel * t_rel + b * t_rel + c;
                    profile.points.push_back({t_rel, vel});
                }
                
                // Ensure last point is exactly at segment_duration
                if (profile.points.back().time < segment_duration) {
                    double t_rel = segment_duration;
                    double vel = a * t_rel * t_rel + b * t_rel + c;
                    profile.points.push_back({t_rel, vel});
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Converted quadratic (a=%.3f, b=%.3f, c=%.3f) to %zu points for %s",
                    a, b, c, profile.points.size(), key.c_str());
                return profile;
            }
        }
        
        // Default fallback
        RCLCPP_WARN(this->get_logger(), 
            "Could not parse '%s', using zero velocity", key.c_str());
        profile.points.push_back({0.0, 0.0});
        return profile;
    }
    
    //
    // SIMULATION LOGIC
    //
    
    /**
     * @brief Find which segment contains the given simulation time
     */
    const TimeSegment* find_segment_at_time(double t) const
    {
        for (const auto& seg : segments_) {
            if (seg.contains_time(t)) {
                return &seg;
            }
        }
        return nullptr;
    }
    
    /**
     * @brief Calculate velocity at current simulation time
     */
    void get_velocity_at_time(double t, double& vx, double& vy, double& omega) const
    {
        const TimeSegment* segment = find_segment_at_time(t);
        
        if (!segment) {
            vx = vy = omega = 0.0;
            return;
        }
        
        // Calculate time relative to segment start
        double t_rel = t - segment->start_time;
        
        // Evaluate each axis using its profile
        vx = segment->velocity_x.evaluate(t_rel);
        vy = segment->velocity_y.evaluate(t_rel);
        omega = segment->omega.evaluate(t_rel);
    }
    
    //
    // TIMER CALLBACKS
    //
    
    /**
     * @brief Main timer callback - called at publish_rate_hz
     */
    void timer_callback()
    {
        // Handle looping if enabled
        if (loop_ && sim_time_ >= total_duration_) {
            sim_time_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "═══ LOOP: Restarting from t=0 ═══");
        }
        
        // Get velocity at current time
        double vx, vy, omega;
        get_velocity_at_time(sim_time_, vx, vy, omega);
        
        // Round to specified decimal places
        vx = std::round(vx * rounding_factor_) / rounding_factor_;
        vy = std::round(vy * rounding_factor_) / rounding_factor_;
        omega = std::round(omega * rounding_factor_) / rounding_factor_;
        
        // Get current ROS time for stamping
        auto current_time = this->now();
        
        // Publish mecanum wheel data
        publish_mecanum_data(vx, vy, omega, current_time);
        
        // Log status every 2 seconds
        log_counter_++;
        if (log_counter_ >= rate_hz_ * 2) {
            const TimeSegment* seg = find_segment_at_time(sim_time_);
            if (seg) {
                RCLCPP_INFO(this->get_logger(), 
                    "[t=%5.1fs] LAGRANGE | vx=%+.3f  vy=%+.3f  ω=%+.3f",
                    sim_time_, vx, vy, omega);
            }
            log_counter_ = 0;
        }
        
        // Advance simulation time
        sim_time_ += dt_;
    }
    
    /**
     * @brief One-shot callback to send initial position reset
     */
    void send_initial_position_reset()
    {
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
        RCLCPP_INFO(this->get_logger(), 
            "Initial position reset: (%.2f, %.2f) α=%.2f°", 
            initial_x_, initial_y_, initial_alpha_ * 180.0 / M_PI);
    }
    
    //
    // MESSAGE PUBLISHING
    //
    
    /**
     * @brief Publish mecanum wheel data
     */
    void publish_mecanum_data(double vx, double vy, double omega, const rclcpp::Time& stamp)
    {
        // Calculate mecanum wheel velocities using inverse kinematics
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
    
    //
    // MEMBER VARIABLES
    //
    
    // Configuration
    std::string path_file_;
    int rate_hz_;
    int decimalen_;
    double rounding_factor_;
    double dt_;
    double total_duration_;
    double initial_x_, initial_y_, initial_alpha_;
    double wheel_radius_, wheel_base_x_, wheel_base_y_;
    bool loop_;
    
    // Simulation state
    double sim_time_{0.0};
    int log_counter_{0};
    std::array<double, 4> wheel_angles_{{0.0, 0.0, 0.0, 0.0}};
    
    // Path definition
    std::vector<TimeSegment> segments_;
    
    // ROS publishers
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reset_pub_;
    
    // ROS timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr initial_reset_timer_;
};

//
// MAIN
//

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MecanumSimulator>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("mecanum_simulator"), 
            "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
