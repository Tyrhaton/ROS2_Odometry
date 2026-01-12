/**
 * @file imu_simulator.cpp
 * @brief IMU Simulator - Lagrange Interpolation Implementation
 *
 * Reads acceleration profiles from YAML files and publishes IMU data.
 * Uses Lagrange interpolation for smooth acceleration profiles.
 *
 * Lagrange interpolation allows defining acceleration values at specific
 * time points, and automatically interpolates between them using polynomial
 * basis functions.
 *
 * @author Group g1 - Lagrange interpolation version
 * @date 2026-01-05
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

/**
 * @struct InterpolationPoint
 * @brief A single data point for Lagrange interpolation
 */
struct InterpolationPoint {
    double time;          // Time value (relative to segment start)
    double acceleration;  // Acceleration value at this time
};

/**
 * @struct AccelerationProfile
 * @brief Lagrange interpolation profile for one axis
 * 
 * Stores a set of (time, acceleration) points and performs
 * Lagrange polynomial interpolation between them.
 */
struct AccelerationProfile {
    std::vector<InterpolationPoint> points;
    
    /**
     * @brief Calculate Lagrange basis polynomial L_i(t)
     * @param t Time value at which to evaluate
     * @param i Index of the basis polynomial
     * @return Value of L_i(t)
     */
    double lagrange_basis(double t, size_t i) const {
        if (i >= points.size()) return 0.0;
        
        double result = 1.0;
        for (size_t j = 0; j < points.size(); ++j) {
            if (i != j) {
                result *= (t - points[j].time) / (points[i].time - points[j].time);
            }
        }
        return result;
    }
    
    /**
     * @brief Calculate acceleration at relative time using Lagrange interpolation
     * @param t_rel Time relative to segment start (t - segment_start_time)
     * @return Interpolated acceleration value at this time
     */
    double evaluate(double t_rel) const {
        // Handle edge cases
        if (points.empty()) {
            return 0.0;
        }
        
        if (points.size() == 1) {
            return points[0].acceleration;
        }
        
        // PIECEWISE LAGRANGE INTERPOLATION
        // Find the interval [xi, xi+1] that contains t_rel
        // Then use only those 2 points for linear interpolation
        
        // If before first point, use first value
        if (t_rel <= points[0].time) {
            return points[0].acceleration;
        }
        
        // If after last point, use last value
        if (t_rel >= points.back().time) {
            return points.back().acceleration;
        }
        
        // Find the right interval
        for (size_t i = 0; i < points.size() - 1; ++i) {
            if (t_rel >= points[i].time && t_rel <= points[i+1].time) {
                // LINEAR PIECEWISE LAGRANGE between points[i] and points[i+1]
                // p1(x) = f(xi) * (x - xi+1)/(xi - xi+1) + f(xi+1) * (x - xi)/(xi+1 - xi)
                
                double xi = points[i].time;
                double xi1 = points[i+1].time;
                double fi = points[i].acceleration;
                double fi1 = points[i+1].acceleration;
                
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
        return points.back().acceleration;
    }
};

/**
 * @struct TimeSegment
 * @brief Defines acceleration behavior during a specific time interval
 */
struct TimeSegment {
    double start_time;  // When this segment begins (seconds)
    double end_time;    // When this segment ends (seconds)
    
    AccelerationProfile accel_x;  // X-axis acceleration profile
    AccelerationProfile accel_y;  // Y-axis acceleration profile
    AccelerationProfile accel_z;  // Z-axis acceleration profile (usually constant)
    
    /**
     * @brief Check if a given time falls within this segment
     */
    bool contains_time(double t) const {
        return (t >= start_time) && (t < end_time);
    }
};

/**
 * @class IMUSimulator
 * @brief Publishes simulated IMU sensor data based on YAML acceleration profiles
 * 
 * Clean, readable implementation with clear separation of concerns.
 */
class IMUSimulator : public rclcpp::Node
{
public:
    IMUSimulator() : Node("imu_simulator")
    {
        // PARAMETER SETUP
        declare_and_get_parameters();
        
        // LOAD PATH FROM YAML
        load_path_from_yaml();
        
        // CREATE PUBLISHERS
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/data", 10);
        accel_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>(
            "/simulator/acceleration", 10);
        reset_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/position/corrected", 10);
        segment_info_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/simulator/segment_info", 10);
        
        // CREATE TIMERS
        // Main timer for publishing IMU data
        double timer_period_ms = 1000.0 / rate_hz_;
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(timer_period_ms)),
            std::bind(&IMUSimulator::timer_callback, this));
        
        // One-shot timer to send initial position reset
        initial_reset_timer_ = this->create_wall_timer(
            500ms,
            std::bind(&IMUSimulator::send_initial_position_reset, this));
        
        // STARTUP INFO
        RCLCPP_INFO(this->get_logger(), "- IMU SIMULATOR STARTED");
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
        this->declare_parameter<int>("interval", -1);  // Sample interval in ms (overrides YAML)
        this->declare_parameter<int>("decimalen", 6);  // Decimal precision for rounding (default: 6 = no visible rounding)
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_alpha", 0.0);
        this->declare_parameter<bool>("loop", false);

        path_file_ = this->get_parameter("path_file").as_string();
        rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
        int interval_ms = this->get_parameter("interval").as_int();
        decimalen_ = this->get_parameter("decimalen").as_int();
        initial_x_ = this->get_parameter("initial_x").as_double();
        initial_y_ = this->get_parameter("initial_y").as_double();
        initial_alpha_ = this->get_parameter("initial_alpha").as_double();
        loop_ = this->get_parameter("loop").as_bool();
        
        // If interval is specified, override rate_hz
        if (interval_ms > 0) {
            rate_hz_ = 1000 / interval_ms;  // Convert ms to Hz
            RCLCPP_INFO(this->get_logger(), 
                "Using interval parameter: %d ms -> %d Hz", interval_ms, rate_hz_);
        }
        
        // Calculate rounding factor based on decimalen
        rounding_factor_ = std::pow(10.0, decimalen_);
        RCLCPP_INFO(this->get_logger(), 
            "Decimal precision: %d decimalen (rounding factor: %.0f)", decimalen_, rounding_factor_);
        
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
            
            // Sort segments by start time (just in case YAML is out of order)
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
        // If already absolute path, return as-is
        if (!file_path.empty() && file_path[0] == '/') {
            return file_path;
        }
        
        // Try to find in package share directory
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry_pkg");
            return pkg_share + "/" + file_path;
        } catch (...) {
            // If package not found, return original path
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
        
        // Parse X-axis acceleration
        seg.accel_x = parse_acceleration_profile(seg_node, "accel_x", segment_duration);
        
        // Parse Y-axis acceleration
        seg.accel_y = parse_acceleration_profile(seg_node, "accel_y", segment_duration);
        
        // Parse Z-axis acceleration (default to zero)
        seg.accel_z = parse_acceleration_profile(seg_node, "accel_z", segment_duration);
        
        return seg;
    }
    
    /**
     * @brief Parse acceleration profile for a single axis using interpolation points
     * 
     * Expected YAML format:
     *   accel_x:
     *     - [time, acceleration]
     *     - [time, acceleration]
     *     ...
     * 
     * Or for constant acceleration:
     *   accel_x: value
     * 
     * @param seg_node The segment YAML node
     * @param key The key to parse (e.g., "accel_x")
     * @param segment_duration Duration of the segment (for proper scaling)
     */
    AccelerationProfile parse_acceleration_profile(
        const YAML::Node& seg_node, 
        const std::string& key,
        double segment_duration)
    {
        AccelerationProfile profile;
        
        if (!seg_node[key]) {
            // No value specified, default to zero at t=0
            profile.points.push_back({0.0, 0.0});
            return profile;
        }
        
        auto accel_node = seg_node[key];
        
        // CASE 1: Simple scalar value (constant acceleration)
        if (accel_node.IsScalar()) {
            double value = accel_node.as<double>();
            // Create two points with same value for constant interpolation
            profile.points.push_back({0.0, value});
            profile.points.push_back({segment_duration, value});
            return profile;
        }
        
        // CASE 2: Sequence of [time, acceleration] pairs
        if (accel_node.IsSequence()) {
            for (const auto& point_node : accel_node) {
                if (point_node.IsSequence() && point_node.size() >= 2) {
                    InterpolationPoint pt;
                    pt.time = point_node[0].as<double>();
                    pt.acceleration = point_node[1].as<double>();
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
        
        // CASE 3: Legacy format support (map with coefficients)
        // This maintains backward compatibility with old YAML files
        if (accel_node.IsMap()) {
            // Try to parse as linear: a(t) = m*t + b
            if (accel_node["m"] && accel_node["b"]) {
                double m = accel_node["m"].as<double>();
                double b = accel_node["b"].as<double>();
                
                // Generate points every 1ms for ultra-smooth interpolation
                // Much denser than publish rate to ensure smooth values at ANY sample rate
                double step = 0.001;  // 1ms between points
                int num_points = std::max(2, static_cast<int>(segment_duration / step) + 1);
                
                for (int i = 0; i < num_points; ++i) {
                    double t_rel = std::min(i * step, segment_duration);
                    double accel = m * t_rel + b;
                    profile.points.push_back({t_rel, accel});
                }
                
                // Ensure last point is exactly at segment_duration
                if (profile.points.back().time < segment_duration) {
                    double accel = m * segment_duration + b;
                    profile.points.push_back({segment_duration, accel});
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Converted legacy linear (m=%.3f, b=%.3f, dur=%.1fs) to %zu points (1ms spacing)",
                    m, b, segment_duration, profile.points.size());
                return profile;
            }
            
            // Try to parse as quadratic: a(t) = a*t² + b*t + c
            if (accel_node["a"] || accel_node["b"] || accel_node["c"]) {
                double a = accel_node["a"] ? accel_node["a"].as<double>() : 0.0;
                double b = accel_node["b"] ? accel_node["b"].as<double>() : 0.0;
                double c = accel_node["c"] ? accel_node["c"].as<double>() : 0.0;
                
                // Generate points every 1ms for ultra-smooth interpolation
                double step = 0.001;  // 1ms between points
                int num_points = std::max(3, static_cast<int>(segment_duration / step) + 1);
                
                for (int i = 0; i < num_points; ++i) {
                    double t_rel = std::min(i * step, segment_duration);
                    double accel = a * t_rel * t_rel + b * t_rel + c;
                    profile.points.push_back({t_rel, accel});
                }
                
                // Ensure last point is exactly at segment_duration
                if (profile.points.back().time < segment_duration) {
                    double t_rel = segment_duration;
                    double accel = a * t_rel * t_rel + b * t_rel + c;
                    profile.points.push_back({t_rel, accel});
                }
                
                RCLCPP_INFO(this->get_logger(), 
                    "Converted legacy quadratic (a=%.3f, b=%.3f, c=%.3f, dur=%.1fs) to %zu points (1ms spacing)",
                    a, b, c, segment_duration, profile.points.size());
                return profile;
            }
        }
        
        // Default fallback
        RCLCPP_WARN(this->get_logger(), 
            "Could not parse '%s', using zero acceleration", key.c_str());
        profile.points.push_back({0.0, 0.0});
        return profile;
    }
    
    //
    // SIMULATION LOGIC
    //
    
    /**
     * @brief Find which segment contains the given simulation time
     * @param t Current simulation time (seconds)
     * @return Pointer to segment, or nullptr if time is outside all segments
     */
    const TimeSegment* find_segment_at_time(double t) const
    {
        for (const auto& seg : segments_) {
            if (seg.contains_time(t)) {
                return &seg;
            }
        }
        
        // If we're past all segments OR before first segment, return nullptr
        // This will cause acceleration to be zero (no active segment)
        return nullptr;
    }
    
    /**
     * @brief Calculate acceleration at current simulation time
     * @param t Current simulation time (seconds)
     * @param ax Output: X-axis acceleration
     * @param ay Output: Y-axis acceleration
     * @param az Output: Z-axis acceleration
     */
    void get_acceleration_at_time(double t, double& ax, double& ay, double& az) const
    {
        const TimeSegment* segment = find_segment_at_time(t);
        
        if (!segment) {
            // No segment found - return zero acceleration
            ax = ay = az = 0.0;
            return;
        }
        
        // Calculate time relative to segment start
        double t_rel = t - segment->start_time;
        
        // Evaluate each axis using its profile
        ax = segment->accel_x.evaluate(t_rel);
        ay = segment->accel_y.evaluate(t_rel);
        az = segment->accel_z.evaluate(t_rel);
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
            RCLCPP_INFO(this->get_logger(), "LOOP: Restarting from t=0");
        }
        
        // Get acceleration at current time
        double ax, ay, az;
        get_acceleration_at_time(sim_time_, ax, ay, az);
        
        // Round to specified decimal places (decimalen parameter)
        // e.g., decimalen=2 -> rounding_factor=100 -> rounds to 0.01
        ax = std::round(ax * rounding_factor_) / rounding_factor_;
        ay = std::round(ay * rounding_factor_) / rounding_factor_;
        az = std::round(az * rounding_factor_) / rounding_factor_;
        
        // Create timestamp from simulation time (starts from 0)
        rclcpp::Time timestamp = rclcpp::Time(static_cast<int64_t>(sim_time_ * 1e9), RCL_ROS_TIME);
        publish_imu_message(ax, ay, az, timestamp);
        
        // Publish segment info (current segment number and total)
        publish_segment_info();
        
        // Log status every 2 seconds
        log_counter_++;
        if (log_counter_ >= rate_hz_ * 2) {
            const TimeSegment* seg = find_segment_at_time(sim_time_);
            if (seg) {
                RCLCPP_INFO(this->get_logger(), 
                    "[t=%5.1fs] LAGRANGE | ax=%+.3f  ay=%+.3f  az=%+.3f",
                    sim_time_, ax, ay, az);
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
        initial_reset_timer_->cancel();  // This is a one-shot timer
        
        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";
        
        // Set initial position
        msg.pose.pose.position.x = initial_x_;
        msg.pose.pose.position.y = initial_y_;
        msg.pose.pose.position.z = 0.0;
        
        // Set initial orientation (yaw)
        msg.pose.pose.orientation.z = std::sin(initial_alpha_ / 2.0);
        msg.pose.pose.orientation.w = std::cos(initial_alpha_ / 2.0);
        
        // Set initial velocity (zero)
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
     * @brief Publish IMU sensor data
     */
    void publish_imu_message(double ax, double ay, double az, const rclcpp::Time& stamp)
    {
        // Publish standard IMU message
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = stamp;
        imu_msg.header.frame_id = "base_link";
        
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;
        
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = 0.0;
        
        imu_msg.orientation_covariance[0] = -1.0;  // Orientation not provided
        
        imu_pub_->publish(imu_msg);
        
        // Also publish as AccelStamped (for convenience)
        auto accel_msg = geometry_msgs::msg::AccelStamped();
        accel_msg.header = imu_msg.header;
        accel_msg.accel.linear.x = ax;
        accel_msg.accel.linear.y = ay;
        accel_msg.accel.linear.z = az;
        
        accel_pub_->publish(accel_msg);
    }
    
    /**
     * @brief Publish current segment info for visualization
     */
    void publish_segment_info()
    {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.resize(2);
        
        // Find current segment index (1-indexed for display)
        int current_seg = 1;
        for (size_t i = 0; i < segments_.size(); ++i) {
            if (segments_[i].contains_time(sim_time_)) {
                current_seg = static_cast<int>(i + 1);
                break;
            }
        }
        
        msg.data[0] = current_seg;
        msg.data[1] = static_cast<int>(segments_.size());
        
        segment_info_pub_->publish(msg);
    }
    
    //
    // UTILITY FUNCTIONS
    //
    
    //
    // MEMBER VARIABLES
    //
    
    // Configuration
    std::string path_file_;
    int rate_hz_;
    double dt_;                    // Time step (1 / rate_hz_)
    double total_duration_;        // Total simulation duration
    double initial_x_, initial_y_, initial_alpha_;
    bool loop_;
    int decimalen_;                // Decimal precision for rounding
    double rounding_factor_;       // 10^decimalen for rounding calculation
    
    // Simulation state
    double sim_time_{0.0};         // Current simulation time
    int log_counter_{0};           // For periodic logging
    
    // Path definition
    std::vector<TimeSegment> segments_;
    
    // ROS publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr accel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr reset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr segment_info_pub_;
    
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
        auto node = std::make_shared<IMUSimulator>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("imu_simulator"), 
            "Fatal error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
