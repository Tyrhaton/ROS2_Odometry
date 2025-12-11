/**
 * @file position_velocity_approximator.cpp
 * @brief Position and velocity approximator using acceleration sensor data
 *
 * This node calculates position and velocity by integrating acceleration data
 * from an IMU sensor. It handles transformation from sensor frame to robot frame
 * and accounts for the coupling between rotation and linear motion.
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "odometry_interfaces_pkg/msg/velocity_data.hpp"
#include "odometry_interfaces_pkg/msg/position_data.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <urdf/model.h>

using namespace std::chrono_literals;

/**
 * @class PositionVelocityApproximator
 * @brief Integrates acceleration data to estimate velocity and position
 *
 * This node subscribes to IMU acceleration data and integrates it numerically
 * to compute velocity and position in the fixed frame (map).
 */
class PositionVelocityApproximator : public rclcpp::Node
{
public:
    PositionVelocityApproximator() : Node("position_velocity_approximator")
    {
        // Declare parameters
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_alpha", 0.0);

        // Drift correction parameters
        this->declare_parameter<double>("static_accel_threshold", 0.5);  // m/s² - threshold for static detection
        this->declare_parameter<double>("static_gyro_threshold", 0.3);   // rad/s - threshold for rotation detection
        this->declare_parameter<double>("velocity_damping", 0.95);       // damping factor when near-static
        this->declare_parameter<double>("gravity_z", 9.81);              // gravity compensation
        this->declare_parameter<bool>("enable_drift_correction", true);  // enable/disable drift correction
        this->declare_parameter<int>("zupt_count_threshold", 25);        // frames static before ZUPT (0.5s @ 50Hz)
        this->declare_parameter<double>("wheel_radius", 0.05);           // mecanum wheel radius [m]
        this->declare_parameter<double>("wheel_base_x", 0.30);           // distance front-rear [m]
        this->declare_parameter<double>("wheel_base_y", 0.25);           // distance left-right [m]
        this->declare_parameter<std::string>("robot_description", "");

        // Get initial position parameters
        position_x_ = this->get_parameter("initial_x").as_double();
        position_y_ = this->get_parameter("initial_y").as_double();
        position_z_ = 0.0;  // Always 0 for 2D motion
        alpha_ = this->get_parameter("initial_alpha").as_double();

        // Get drift correction parameters
        static_accel_threshold_ = this->get_parameter("static_accel_threshold").as_double();
        static_gyro_threshold_ = this->get_parameter("static_gyro_threshold").as_double();
        velocity_damping_ = this->get_parameter("velocity_damping").as_double();
        gravity_z_ = this->get_parameter("gravity_z").as_double();
        enable_drift_correction_ = this->get_parameter("enable_drift_correction").as_bool();
        zupt_count_threshold_ = this->get_parameter("zupt_count_threshold").as_int();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        lx_ = this->get_parameter("wheel_base_x").as_double() / 2.0;
        ly_ = this->get_parameter("wheel_base_y").as_double() / 2.0;
        parse_urdf_geometry();  // override lx_, ly_, wheel_radius_ when available

        // Initialize velocities to zero
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
        velocity_z_ = 0.0;
        angular_velocity_z_ = 0.0;

        // Initialize ZUPT counter
        static_count_ = 0;

        // Initialize previous time
        last_time_valid_ = false;

        // Create subscribers
        // Sensor QoS to match IMU publishers (best effort, shallow history).
        auto sensor_qos = rclcpp::SensorDataQoS();
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data",
            sensor_qos,
            std::bind(&PositionVelocityApproximator::imu_callback, this, std::placeholders::_1));

        // Create subscriber for position reset (from position determinator)
        position_reset_sub_ = this->create_subscription<odometry_interfaces_pkg::msg::PositionData>(
            "/position/corrected",
            10,
            std::bind(&PositionVelocityApproximator::position_reset_callback, this, std::placeholders::_1));

        // Create publishers
        velocity_pub_ = this->create_publisher<odometry_interfaces_pkg::msg::VelocityData>(
            "/odometry/velocity_from_accel", 10);

        position_pub_ = this->create_publisher<odometry_interfaces_pkg::msg::PositionData>(
            "/odometry/position_from_accel", 10);
        wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/odometry/wheel_speeds_from_accel", 10);
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "Position/Velocity Approximator started");
        RCLCPP_INFO(this->get_logger(), "Initial position: (%.3f, %.3f, %.3f), alpha: %.3f rad",
                    position_x_, position_y_, position_z_, alpha_);

        if (enable_drift_correction_) {
            RCLCPP_INFO(this->get_logger(), "Drift correction ENABLED:");
            RCLCPP_INFO(this->get_logger(), "  - Static accel threshold: %.3f m/s²", static_accel_threshold_);
            RCLCPP_INFO(this->get_logger(), "  - Static gyro threshold: %.3f rad/s", static_gyro_threshold_);
            RCLCPP_INFO(this->get_logger(), "  - Velocity damping: %.3f", velocity_damping_);
            RCLCPP_INFO(this->get_logger(), "  - Gravity compensation: %.3f m/s²", gravity_z_);
            RCLCPP_INFO(this->get_logger(), "  - ZUPT threshold: %d frames (%.1f sec @ 50Hz)",
                        zupt_count_threshold_, zupt_count_threshold_ / 50.0);
        } else {
            RCLCPP_WARN(this->get_logger(), "Drift correction DISABLED - expect significant drift!");
        }
    }

private:
    /**
     * @brief Callback for position reset messages
     * @param msg Corrected position from position determinator
     * 
     * If position values are NaN, only velocity is updated (keep current position).
     * Otherwise, both position and velocity are reset.
     */
    void position_reset_callback(const odometry_interfaces_pkg::msg::PositionData::SharedPtr msg)
    {
        // Check if this is a velocity-only update (NaN position values)
        bool velocity_only = std::isnan(msg->x) || std::isnan(msg->y);
        
        if (velocity_only) {
            // Velocity-only update: keep current position, just update velocities
            RCLCPP_INFO(this->get_logger(), 
                "Velocity update: vx=%.3f -> %.3f, vy=%.3f -> %.3f (position unchanged)",
                velocity_x_, msg->initial_vx, velocity_y_, msg->initial_vy);
            
            velocity_x_ = msg->initial_vx;
            velocity_y_ = msg->initial_vy;
            velocity_z_ = 0.0;
            angular_velocity_z_ = 0.0;
        } else {
            // Full reset: update both position and velocity
            RCLCPP_INFO(this->get_logger(), "Resetting position to (%.3f, %.3f, %.3f), alpha: %.3f",
                        msg->x, msg->y, msg->z, msg->alpha);
            RCLCPP_INFO(this->get_logger(), "Initial velocities: vx=%.3f, vy=%.3f",
                        msg->initial_vx, msg->initial_vy);

            // Reset position and orientation
            position_x_ = msg->x;
            position_y_ = msg->y;
            position_z_ = msg->z;
            alpha_ = msg->alpha;

            // Set initial velocities from message
            velocity_x_ = msg->initial_vx;
            velocity_y_ = msg->initial_vy;
            velocity_z_ = 0.0;
            angular_velocity_z_ = 0.0;
        }

        // Invalidate last time and previous acceleration to restart integration
        last_time_valid_ = false;
        have_prev_accel_ = false;
    }

    /**
     * @brief Callback for IMU data
     * @param msg IMU message with linear acceleration and angular velocity
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Get current time (fall back to node clock if header stamp is zero or non-increasing)
        rclcpp::Time current_time = msg->header.stamp;
        if (current_time.nanoseconds() == 0) {
            current_time = this->now();
        }

        // Skip first message (need two for dt calculation)
        if (!last_time_valid_) {
            last_time_ = current_time;
            last_time_valid_ = true;
            return;
        }

        // Calculate time delta; if non-positive, fallback to wall-clock to avoid getting stuck
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0) {
            current_time = this->now();
            dt = (current_time - last_time_).seconds();
        }
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid dt: %.3f, skipping", dt);
            last_time_ = current_time;
            return;
        }

        // Extract acceleration and angular velocity from IMU
        // Note: IMU data is in sensor frame, we need to transform to map frame
        double accel_x_sensor = msg->linear_acceleration.x;
        double accel_y_sensor = msg->linear_acceleration.y;
        double accel_z_sensor = msg->linear_acceleration.z;
        double angular_vel_z = msg->angular_velocity.z;

        // Gravity compensation: Remove gravity component from Z-axis
        // When IMU is flat, Z should read ~9.81 m/s² due to gravity
        // Subtract gravity to get actual acceleration
        double accel_z_compensated = accel_z_sensor - gravity_z_;

        // Rotate accelerations from body/sensor frame to map frame using current yaw.
        const double cos_alpha = std::cos(alpha_);
        const double sin_alpha = std::sin(alpha_);
        double accel_x_map = cos_alpha * accel_x_sensor - sin_alpha * accel_y_sensor;
        double accel_y_map = sin_alpha * accel_x_sensor + cos_alpha * accel_y_sensor;
        double accel_z_map = accel_z_compensated;

        // Calculate acceleration magnitude for static detection
        // IMPORTANT: Only use XY for 2D planar motion - Z has gravity compensation artifacts
        double accel_magnitude = std::sqrt(accel_x_map * accel_x_map +
                                           accel_y_map * accel_y_map);

        // Calculate angular velocity magnitude
        double gyro_magnitude = std::abs(angular_vel_z);

        // Detect if sensor is static (no significant movement)
        bool is_static = (accel_magnitude < static_accel_threshold_) &&
                         (gyro_magnitude < static_gyro_threshold_);

        // Store old velocity for trapezoidal position integration
        double old_velocity_x = velocity_x_;
        double old_velocity_y = velocity_y_;

        if (enable_drift_correction_ && is_static) {
            // Increment static counter
            static_count_++;

            // ZUPT (Zero Velocity Update): If static for long enough, HARD reset velocity to 0
            if (static_count_ >= zupt_count_threshold_) {
                // HARD RESET velocity to zero
                velocity_x_ = 0.0;
                velocity_y_ = 0.0;
                velocity_z_ = 0.0;

                // Log ZUPT activation (but not every frame to avoid spam)
                if (static_count_ == zupt_count_threshold_) {
                    RCLCPP_INFO(this->get_logger(), "ZUPT ACTIVE - Velocity LOCKED to ZERO");
                }
            } else {
                // Still counting towards ZUPT, apply damping
                velocity_x_ *= velocity_damping_;
                velocity_y_ *= velocity_damping_;
                velocity_z_ *= velocity_damping_;

                // If velocity is very small, set to zero to prevent creep
                if (std::abs(velocity_x_) < 0.01) velocity_x_ = 0.0;
                if (std::abs(velocity_y_) < 0.01) velocity_y_ = 0.0;
                if (std::abs(velocity_z_) < 0.01) velocity_z_ = 0.0;
            }

            // Don't integrate acceleration when static (it's just noise)
            // Keep angular velocity for orientation
            angular_velocity_z_ = angular_vel_z;
        } else {
            // Sensor is moving - reset static counter
            if (static_count_ > 0) {
                RCLCPP_INFO(this->get_logger(), "ZUPT RELEASED - Movement detected");
                static_count_ = 0;
            }

            // ===== TRAPEZOIDAL INTEGRATION FOR VELOCITY =====
            // v(t+dt) = v(t) + (a(t) + a(t+dt)) * dt / 2
            // This is more accurate than Euler: v += a * dt
            if (have_prev_accel_) {
                velocity_x_ += (prev_accel_x_map_ + accel_x_map) * dt * 0.5;
                velocity_y_ += (prev_accel_y_map_ + accel_y_map) * dt * 0.5;
            } else {
                // First sample: use simple Euler as fallback
                velocity_x_ += accel_x_map * dt;
                velocity_y_ += accel_y_map * dt;
            }
            angular_velocity_z_ = angular_vel_z;  // Angular velocity from sensor
        }

        // ===== TRAPEZOIDAL INTEGRATION FOR POSITION =====
        // p(t+dt) = p(t) + (v(t) + v(t+dt)) * dt / 2
        // This is more accurate than Euler: p += v * dt
        position_x_ += (old_velocity_x + velocity_x_) * dt * 0.5;
        position_y_ += (old_velocity_y + velocity_y_) * dt * 0.5;

        // ===== TRAPEZOIDAL INTEGRATION FOR ORIENTATION =====
        // alpha(t+dt) = alpha(t) + (omega(t) + omega(t+dt)) * dt / 2
        if (have_prev_accel_) {
            alpha_ += (prev_angular_vel_z_ + angular_velocity_z_) * dt * 0.5;
        } else {
            alpha_ += angular_velocity_z_ * dt;
        }

        // Store current values for next iteration's trapezoidal integration
        prev_accel_x_map_ = accel_x_map;
        prev_accel_y_map_ = accel_y_map;
        prev_angular_vel_z_ = angular_velocity_z_;
        have_prev_accel_ = true;

        // Normalize alpha to [-pi, pi]
        alpha_ = std::atan2(std::sin(alpha_), std::cos(alpha_));

        // Publish velocity
        auto vel_msg = odometry_interfaces_pkg::msg::VelocityData();
        vel_msg.header.stamp = current_time;
        vel_msg.header.frame_id = "map";
        vel_msg.linear_x = velocity_x_;
        vel_msg.linear_y = velocity_y_;
        vel_msg.linear_z = velocity_z_;
        vel_msg.angular_z = angular_velocity_z_;
        velocity_pub_->publish(vel_msg);

        // Compute wheel angular velocities from robot-frame velocities
        double vx_robot =  cos_alpha * velocity_x_ + sin_alpha * velocity_y_;
        double vy_robot = -sin_alpha * velocity_x_ + cos_alpha * velocity_y_;
        double k = lx_ + ly_;
        double inv_r = (wheel_radius_ > 1e-6) ? (1.0 / wheel_radius_) : 0.0;
        std::array<double,4> w{};
        w[0] = inv_r * ( vx_robot - vy_robot - k * angular_velocity_z_); // FL
        w[1] = inv_r * ( vx_robot + vy_robot + k * angular_velocity_z_); // FR
        w[2] = inv_r * ( vx_robot + vy_robot - k * angular_velocity_z_); // RL
        w[3] = inv_r * ( vx_robot - vy_robot + k * angular_velocity_z_); // RR
        for (size_t i = 0; i < 4; ++i) {
            wheel_angle_[i] += w[i] * dt;
        }
        std_msgs::msg::Float64MultiArray wheel_msg;
        wheel_msg.data = {w[0], w[1], w[2], w[3]};
        wheel_vel_pub_->publish(wheel_msg);

        // Publish joint states for wheel rotation visualization
        sensor_msgs::msg::JointState js;
        js.header.stamp = current_time;
        js.name = {"wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"};
        js.position = {wheel_angle_[0], wheel_angle_[1], wheel_angle_[2], wheel_angle_[3]};
        js.velocity = {w[0], w[1], w[2], w[3]};
        joint_state_pub_->publish(js);

        // Publish position
        auto pos_msg = odometry_interfaces_pkg::msg::PositionData();
        pos_msg.header.stamp = current_time;
        pos_msg.header.frame_id = "map";
        pos_msg.x = position_x_;
        pos_msg.y = position_y_;
        pos_msg.z = position_z_;
        pos_msg.alpha = alpha_;
        position_pub_->publish(pos_msg);

        // Log position every second for monitoring
        static int log_counter = 0;
        if (++log_counter >= 50) {  // Assuming ~50Hz update rate
            if (enable_drift_correction_) {
                const char* status = is_static ?
                    (static_count_ >= zupt_count_threshold_ ? "ZUPT-LOCKED" : "STATIC-DAMPING") :
                    "MOVING";
                RCLCPP_INFO(this->get_logger(), "Pos: (%.2f, %.2f) α=%.0f° | Vel: (%.3f, %.3f) | %s | a=%.2f g=%.2f",
                            position_x_, position_y_, alpha_ * 180.0 / M_PI,
                            velocity_x_, velocity_y_,
                            status, accel_magnitude, gyro_magnitude);
            } else {
                RCLCPP_INFO(this->get_logger(), "Position: (%.3f, %.3f, %.3f) α=%.3f° | Velocity: (%.3f, %.3f) ω=%.3f",
                            position_x_, position_y_, position_z_, alpha_ * 180.0 / M_PI,
                            velocity_x_, velocity_y_, angular_velocity_z_);
            }
            log_counter = 0;
        }

        // Update last time
        last_time_ = current_time;
    }

    void parse_urdf_geometry()
    {
        std::string urdf_string = this->get_parameter("robot_description").as_string();
        if (urdf_string.empty()) {
            RCLCPP_WARN(this->get_logger(), "robot_description not provided; using configured wheel params (r=%.3f, lx=%.3f, ly=%.3f)",
                        wheel_radius_, lx_, ly_);
            return;
        }

        urdf::Model model;
        if (!model.initString(urdf_string)) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse URDF from robot_description; keeping configured wheel params.");
            return;
        }

        auto extract_xy = [&](const std::string &joint_name, double &x, double &y) -> bool {
            auto joint = model.getJoint(joint_name);
            if (!joint) return false;
            x = joint->parent_to_joint_origin_transform.position.x;
            y = joint->parent_to_joint_origin_transform.position.y;
            return true;
        };

        double x_fl, y_fl, x_fr, y_fr, x_rl, y_rl, x_rr, y_rr;
        bool ok_fl = extract_xy("wheel_fl_joint", x_fl, y_fl);
        bool ok_fr = extract_xy("wheel_fr_joint", x_fr, y_fr);
        bool ok_rl = extract_xy("wheel_rl_joint", x_rl, y_rl);
        bool ok_rr = extract_xy("wheel_rr_joint", x_rr, y_rr);

        if (ok_fl && ok_fr && ok_rl && ok_rr) {
            double lx_calc = (std::abs(x_fl - x_rl) + std::abs(x_fr - x_rr)) * 0.25;
            double ly_calc = (std::abs(y_fl - y_fr) + std::abs(y_rl - y_rr)) * 0.25;
            if (lx_calc > 1e-4 && ly_calc > 1e-4) {
                lx_ = lx_calc;
                ly_ = ly_calc;
                RCLCPP_INFO(this->get_logger(), "URDF geometry: lx=%.3f, ly=%.3f (half distances)", lx_, ly_);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "URDF missing wheel joints; using configured lx/ly (%.3f/%.3f)", lx_, ly_);
        }

        auto wheel_link = model.getLink("wheel_fl");
        if (wheel_link && wheel_link->collision && wheel_link->collision->geometry) {
            if (wheel_link->collision->geometry->type == urdf::Geometry::CYLINDER) {
                auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(wheel_link->collision->geometry);
                if (cyl && cyl->radius > 1e-4) {
                    wheel_radius_ = cyl->radius;
                    RCLCPP_INFO(this->get_logger(), "URDF wheel radius from collision cylinder: %.3f", wheel_radius_);
                }
            }
        }
    }

    // Member variables - State
    double position_x_, position_y_, position_z_;  ///< Current position in map frame [m]
    double alpha_;                                  ///< Current orientation [rad]
    double velocity_x_, velocity_y_, velocity_z_;  ///< Current velocity in map frame [m/s]
    double angular_velocity_z_;                     ///< Current angular velocity [rad/s]

    // Previous values for trapezoidal integration
    double prev_accel_x_map_{0.0};
    double prev_accel_y_map_{0.0};
    double prev_angular_vel_z_{0.0};
    bool have_prev_accel_{false};

    // Wheel geometry
    double wheel_radius_;
    double lx_;
    double ly_;
    std::array<double, 4> wheel_angle_{{0.0, 0.0, 0.0, 0.0}};

    // Drift correction parameters
    double static_accel_threshold_;   ///< Acceleration threshold for static detection [m/s²]
    double static_gyro_threshold_;    ///< Gyro threshold for static detection [rad/s]
    double velocity_damping_;         ///< Velocity damping factor when near-static
    double gravity_z_;                ///< Gravity compensation value [m/s²]
    bool enable_drift_correction_;    ///< Enable/disable drift correction
    int zupt_count_threshold_;        ///< Number of static frames before ZUPT activates
    int static_count_;                ///< Counter for consecutive static frames

    // Time tracking
    rclcpp::Time last_time_;
    bool last_time_valid_;

    // ROS communication
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<odometry_interfaces_pkg::msg::PositionData>::SharedPtr position_reset_sub_;
    rclcpp::Publisher<odometry_interfaces_pkg::msg::VelocityData>::SharedPtr velocity_pub_;
    rclcpp::Publisher<odometry_interfaces_pkg::msg::PositionData>::SharedPtr position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionVelocityApproximator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
