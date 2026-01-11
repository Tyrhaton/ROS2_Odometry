/**
 * @file mecanum_position_approximator.cpp
 * @brief Position approximator using mecanum wheel encoder velocities
 *
 * This node calculates position by integrating mecanum wheel velocities.
 * It uses the inverse kinematics of mecanum wheels to compute robot velocity
 * in the robot frame, then transforms to the map frame for position estimation.
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <cmath>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/**
 * @class MecanumPositionApproximator
 * @brief Computes position from mecanum wheel encoder velocities
 *
 * This node subscribes to wheel velocities and uses mecanum wheel kinematics
 * to compute robot velocity, then integrates to estimate position.
 */
class MecanumPositionApproximator : public rclcpp::Node
{
public:
    MecanumPositionApproximator() : Node("mecanum_position_approximator")
    {
        // Declare parameters
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_alpha", 0.0);
        this->declare_parameter<double>("wheel_radius", 0.05);      // meters
        this->declare_parameter<double>("wheel_base_x", 0.30);      // distance between front and rear wheels [m]
        this->declare_parameter<double>("wheel_base_y", 0.25);      // distance between left and right wheels [m]

        // Get initial position parameters
        position_x_ = this->get_parameter("initial_x").as_double();
        position_y_ = this->get_parameter("initial_y").as_double();
        position_z_ = 0.0;  // Always 0 for 2D motion
        alpha_ = this->get_parameter("initial_alpha").as_double();

        // Get robot geometry parameters
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        lx_ = this->get_parameter("wheel_base_x").as_double() / 2.0;  // Half distance
        ly_ = this->get_parameter("wheel_base_y").as_double() / 2.0;  // Half distance

        // Initialize velocities to zero
        velocity_x_robot_ = 0.0;
        velocity_y_robot_ = 0.0;
        angular_velocity_z_ = 0.0;

        // Initialize previous time
        last_time_valid_ = false;

        // Create subscribers
        // Expected format: [v_wheel1, v_wheel2, v_wheel3, v_wheel4] in rad/s
        // Wheel numbering: 1=front-left, 2=front-right, 3=rear-right, 4=rear-left
        wheel_vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/wheel_encoders/velocities",
            10,
            std::bind(&MecanumPositionApproximator::wheel_velocity_callback, this, std::placeholders::_1));

        // Create subscriber for position reset
        position_reset_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/position/corrected",
            10,
            std::bind(&MecanumPositionApproximator::position_reset_callback, this, std::placeholders::_1));

        // Create publishers
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/odometry/velocity_from_wheels", 10);

        position_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odometry/position_from_wheels", 10);

        RCLCPP_INFO(this->get_logger(), "Mecanum Position Approximator started");
        RCLCPP_INFO(this->get_logger(), "Initial position: (%.3f, %.3f, %.3f), alpha: %.3f rad",
                    position_x_, position_y_, position_z_, alpha_);
        RCLCPP_INFO(this->get_logger(), "Robot geometry: wheel_radius=%.3fm, lx=%.3fm, ly=%.3fm",
                    wheel_radius_, lx_, ly_);
    }

private:
    /**
     * @brief Callback for position reset messages
     * @param msg Corrected position from position determinator
     */
    void position_reset_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position from pose
        double pos_x = msg->pose.pose.position.x;
        double pos_y = msg->pose.pose.position.y;
        double pos_z = msg->pose.pose.position.z;
        
        // Extract yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double alpha = 2.0 * std::atan2(qz, qw);
        
        RCLCPP_INFO(this->get_logger(), "Resetting position to (%.3f, %.3f, %.3f), alpha: %.3f",
                    pos_x, pos_y, pos_z, alpha);

        // Reset position and orientation
        position_x_ = pos_x;
        position_y_ = pos_y;
        position_z_ = pos_z;
        alpha_ = alpha;

        // Invalidate last time to restart integration
        last_time_valid_ = false;
    }

    /**
     * @brief Compute robot velocity from wheel velocities using mecanum kinematics
     * @param wheel_vels Array of 4 wheel angular velocities [rad/s]
     * @param[out] vx Robot linear velocity in x direction [m/s]
     * @param[out] vy Robot linear velocity in y direction [m/s]
     * @param[out] omega Robot angular velocity [rad/s]
     */
    void compute_robot_velocity(const std::array<double, 4>& wheel_vels,
                                 double& vx, double& vy, double& omega)
    {
        // Mecanum wheel inverse kinematics
        // Wheel configuration (standard mecanum setup):
        //   1 (FL)  2 (FR)
        //   4 (RL)  3 (RR)
        //
        // Forward kinematics (used in simulator):
        //   w1 = (vx - vy - k*omega) / r  # FL
        //   w2 = (vx + vy + k*omega) / r  # FR
        //   w3 = (vx - vy + k*omega) / r  # RR
        //   w4 = (vx + vy - k*omega) / r  # RL
        //
        // Inverse kinematics (solving for vx, vy, omega):
        //   vx    = r/4 * (w1 + w2 + w3 + w4)
        //   vy    = r/4 * (-w1 + w2 - w3 + w4)
        //   omega = r/(4*k) * (-w1 + w2 + w3 - w4)
        //
        // where r is wheel radius, k = lx + ly

        double w1 = wheel_vels[0];  // Front-left
        double w2 = wheel_vels[1];  // Front-right
        double w3 = wheel_vels[2];  // Rear-right
        double w4 = wheel_vels[3];  // Rear-left

        vx = (wheel_radius_ / 4.0) * (w1 + w2 + w3 + w4);
        vy = (wheel_radius_ / 4.0) * (-w1 + w2 - w3 + w4);
        omega = (wheel_radius_ / (4.0 * (lx_ + ly_))) * (-w1 + w2 + w3 - w4);
    }

    /**
     * @brief Callback for wheel velocity messages
     * @param msg Array of wheel velocities [rad/s]
     */
    void wheel_velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Validate message format
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Expected 4 wheel velocities, got %zu", msg->data.size());
            return;
        }

        // Get current time
        rclcpp::Time current_time = this->now();

        // Skip first message (need two for dt calculation)
        if (!last_time_valid_) {
            last_time_ = current_time;
            last_time_valid_ = true;
            return;
        }

        // Calculate time delta
        double dt = (current_time - last_time_).seconds();
        if (dt <= 0.0 || dt > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid dt: %.3f, skipping", dt);
            last_time_ = current_time;
            return;
        }

        // Extract wheel velocities
        std::array<double, 4> wheel_vels = {msg->data[0], msg->data[1], msg->data[2], msg->data[3]};

        // Compute robot velocity in robot frame
        compute_robot_velocity(wheel_vels, velocity_x_robot_, velocity_y_robot_, angular_velocity_z_);

        // Transform robot velocity to map frame using current orientation
        double cos_alpha = std::cos(alpha_);
        double sin_alpha = std::sin(alpha_);

        double velocity_x_map = cos_alpha * velocity_x_robot_ - sin_alpha * velocity_y_robot_;
        double velocity_y_map = sin_alpha * velocity_x_robot_ + cos_alpha * velocity_y_robot_;

        // Integrate velocity to get position (in map frame)
        // p(t+dt) = p(t) + v(t) * dt
        position_x_ += velocity_x_map * dt;
        position_y_ += velocity_y_map * dt;

        // Integrate angular velocity to get orientation
        // alpha(t+dt) = alpha(t) + omega(t) * dt
        alpha_ += angular_velocity_z_ * dt;

        // Normalize alpha to [-pi, pi]
        alpha_ = std::atan2(std::sin(alpha_), std::cos(alpha_));

        // Publish velocity (in map frame)
        auto vel_msg = geometry_msgs::msg::TwistStamped();
        vel_msg.header.stamp = current_time;
        vel_msg.header.frame_id = "map";
        vel_msg.twist.linear.x = velocity_x_map;
        vel_msg.twist.linear.y = velocity_y_map;
        vel_msg.twist.linear.z = 0.0;
        vel_msg.twist.angular.z = angular_velocity_z_;
        velocity_pub_->publish(vel_msg);

        // Publish position as Odometry
        auto pos_msg = nav_msgs::msg::Odometry();
        pos_msg.header.stamp = current_time;
        pos_msg.header.frame_id = "map";
        pos_msg.child_frame_id = "base_link";
        pos_msg.pose.pose.position.x = position_x_;
        pos_msg.pose.pose.position.y = position_y_;
        pos_msg.pose.pose.position.z = position_z_;
        pos_msg.pose.pose.orientation.z = std::sin(alpha_ / 2.0);
        pos_msg.pose.pose.orientation.w = std::cos(alpha_ / 2.0);
        pos_msg.twist.twist.linear.x = velocity_x_map;
        pos_msg.twist.twist.linear.y = velocity_y_map;
        pos_msg.twist.twist.angular.z = angular_velocity_z_;
        position_pub_->publish(pos_msg);

        // Log position every second for monitoring
        static int log_counter = 0;
        if (++log_counter >= 50) {  // Assuming ~50Hz update rate
            RCLCPP_INFO(this->get_logger(), "Position: (%.3f, %.3f, %.3f) α=%.3f° | Velocity: (%.3f, %.3f) ω=%.3f",
                        position_x_, position_y_, position_z_, alpha_ * 180.0 / M_PI,
                        velocity_x_robot_, velocity_y_robot_, angular_velocity_z_);
            log_counter = 0;
        }

        // Update last time
        last_time_ = current_time;
    }

    // Member variables - Robot geometry
    double wheel_radius_;  ///< Radius of mecanum wheels [m]
    double lx_;            ///< Half distance between front and rear wheels [m]
    double ly_;            ///< Half distance between left and right wheels [m]

    // Member variables - State
    double position_x_, position_y_, position_z_;    ///< Current position in map frame [m]
    double alpha_;                                    ///< Current orientation [rad]
    double velocity_x_robot_, velocity_y_robot_;     ///< Current velocity in robot frame [m/s]
    double angular_velocity_z_;                       ///< Current angular velocity [rad/s]

    // Time tracking
    rclcpp::Time last_time_;
    bool last_time_valid_;

    // ROS communication
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr position_reset_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MecanumPositionApproximator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
