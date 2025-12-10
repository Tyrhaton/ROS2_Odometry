/**
 * @file sensor_data_simulator.cpp
 * @brief Sensor data simulator for testing odometry approximation
 *
 * This node simulates acceleration data with multiple modes:
 * 
 * === POSITION RESULT MODES (what you SEE in PlotJuggler) ===
 * - "horizontal":  Position = horizontal line (vx constant, vy=0) -> Y stays constant
 * - "diagonal":    Position = diagonal line (vx=vy constant) -> 45 degree line
 * - "parabola":    Position = PARABOLA in XY plot (X linear, Y quadratic)
 * 
 * === POLYNOMIAL ACCELERATION MODES (Assignment 4 formulas) ===
 * - "accel_constant":  a(t) = c     -> v = linear,    p = QUADRATIC
 * - "accel_linear":    a(t) = m*t   -> v = quadratic, p = CUBIC  
 * - "accel_quadratic": a(t) = k*t²  -> v = cubic,     p = 4th DEGREE
 *
 * The interpolation_mode parameter can be changed at runtime.
 * When switching modes, velocities are reset to match the new mode!
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "odometry_interfaces_pkg/msg/acceleration_data.hpp"
#include "odometry_interfaces_pkg/msg/velocity_data.hpp"
#include "odometry_interfaces_pkg/msg/position_data.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;

/**
 * @class SensorDataSimulator
 * @brief ROS2 node that simulates sensor data for odometry testing
 */
class SensorDataSimulator : public rclcpp::Node
{
public:
    SensorDataSimulator() : Node("sensor_data_simulator")
    {
        // Declare parameters
        this->declare_parameter<std::string>("data_type", "acceleration");
        this->declare_parameter<int>("publish_rate_hz", 50);
        this->declare_parameter<std::string>("interpolation_mode", "horizontal");
        this->declare_parameter<double>("amplitude", 1.0);
        this->declare_parameter<bool>("reset_on_mode_change", false);

        // Get parameters
        data_type_ = this->get_parameter("data_type").as_string();
        int rate_hz = this->get_parameter("publish_rate_hz").as_int();
        interpolation_mode_ = this->get_parameter("interpolation_mode").as_string();
        amplitude_ = this->get_parameter("amplitude").as_double();
        reset_on_mode_change_ = this->get_parameter("reset_on_mode_change").as_bool();

        // Validate interpolation mode
        if (interpolation_mode_ != "horizontal" && 
            interpolation_mode_ != "diagonal" && 
            interpolation_mode_ != "parabola" &&
            interpolation_mode_ != "accel_constant" &&
            interpolation_mode_ != "accel_linear" &&
            interpolation_mode_ != "accel_quadratic") {
            RCLCPP_WARN(this->get_logger(), 
                "Invalid interpolation_mode '%s', defaulting to 'horizontal'", 
                interpolation_mode_.c_str());
            interpolation_mode_ = "horizontal";
        }

        // Register parameter callback for runtime changes
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&SensorDataSimulator::on_parameter_change, this, std::placeholders::_1));

        // Create publishers based on data type
        if (data_type_ == "acceleration") {
            acc_pub_ = this->create_publisher<odometry_interfaces_pkg::msg::AccelerationData>(
                "/simulator/acceleration", 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
                "/imu/data", 10);
        } else if (data_type_ == "velocity") {
            vel_pub_ = this->create_publisher<odometry_interfaces_pkg::msg::VelocityData>(
                "/simulator/velocity", 10);
        }

        // Create publisher for position reset (to reset approximator on mode change)
        reset_pub_ = this->create_publisher<odometry_interfaces_pkg::msg::PositionData>(
            "/position/corrected", 10);

        // Record start time
        start_time_ = this->now();

        // Create timer for publishing
        auto period = std::chrono::milliseconds(1000 / rate_hz);
        timer_ = this->create_wall_timer(
            period,
            std::bind(&SensorDataSimulator::publish_data, this));

        // Create a one-shot timer to send initial reset after startup
        // This ensures the approximator gets the initial velocity settings
        startup_timer_ = this->create_wall_timer(
            500ms,  // Wait 500ms for all nodes to be ready
            std::bind(&SensorDataSimulator::send_startup_reset, this));

        RCLCPP_INFO(this->get_logger(), 
            "Sensor Data Simulator started: data_type=%s, rate=%dHz, mode=%s, amplitude=%.2f",
            data_type_.c_str(), rate_hz, interpolation_mode_.c_str(), amplitude_);
        
        log_expected_behavior();
    }

private:
    /**
     * @brief One-shot callback to send initial reset after startup
     */
    void send_startup_reset()
    {
        // Cancel this timer so it only runs once
        startup_timer_->cancel();
        
        // Reset start time and send initial velocities
        start_time_ = this->now();
        publish_reset();
        
        RCLCPP_INFO(this->get_logger(), "Startup reset sent - beginning simulation");
    }

    /**
     * @brief Log the expected behavior for the current mode
     */
    void log_expected_behavior()
    {
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        // Position result modes - named by what you SEE
        if (interpolation_mode_ == "horizontal") {
            RCLCPP_INFO(this->get_logger(), "Mode: HORIZONTAL - Y stays constant!");
            RCLCPP_INFO(this->get_logger(), "  -> Acceleration: ax=0, ay=0");
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: vx=%.2f, vy=0 (constant)", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Position: X increases linearly, Y flat");
        } 
        else if (interpolation_mode_ == "diagonal") {
            RCLCPP_INFO(this->get_logger(), "Mode: DIAGONAL - 45 degree line!");
            RCLCPP_INFO(this->get_logger(), "  -> Acceleration: ax=0, ay=0");
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: vx=%.2f, vy=%.2f (constant)", amplitude_, amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Position: X and Y increase together");
        } 
        else if (interpolation_mode_ == "parabola") {
            RCLCPP_INFO(this->get_logger(), "Mode: PARABOLA - Curved trajectory!");
            RCLCPP_INFO(this->get_logger(), "  -> Acceleration: ax=0, ay=%.2f (gravity-like)", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: vx=%.2f (constant), vy increases", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Position: X linear, Y quadratic");
        }
        // Assignment polynomial acceleration modes
        else if (interpolation_mode_ == "accel_constant") {
            RCLCPP_INFO(this->get_logger(), "Mode: ACCEL_CONSTANT - a(t) = %.2f", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: LINEAR (increases steadily)");
            RCLCPP_INFO(this->get_logger(), "  -> Position: QUADRATIC curve");
        }
        else if (interpolation_mode_ == "accel_linear") {
            RCLCPP_INFO(this->get_logger(), "Mode: ACCEL_LINEAR - a(t) = %.2f * t", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: QUADRATIC (accelerating faster)");
            RCLCPP_INFO(this->get_logger(), "  -> Position: CUBIC curve");
        }
        else if (interpolation_mode_ == "accel_quadratic") {
            RCLCPP_INFO(this->get_logger(), "Mode: ACCEL_QUADRATIC - a(t) = %.2f * t^2", amplitude_);
            RCLCPP_INFO(this->get_logger(), "  -> Velocity: CUBIC");
            RCLCPP_INFO(this->get_logger(), "  -> Position: 4th DEGREE curve");
        }
        
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    /**
     * @brief Callback for runtime parameter changes
     */
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
            if (param.get_name() == "interpolation_mode") {
                std::string new_mode = param.as_string();
                if (new_mode != "horizontal" && new_mode != "diagonal" && 
                    new_mode != "parabola" &&
                    new_mode != "accel_constant" && new_mode != "accel_linear" && 
                    new_mode != "accel_quadratic") {
                    result.successful = false;
                    result.reason = "interpolation_mode must be: horizontal, diagonal, parabola, accel_constant, accel_linear, or accel_quadratic";
                    return result;
                }
                
                if (new_mode != interpolation_mode_) {
                    RCLCPP_INFO(this->get_logger(), 
                        "Switching interpolation mode: %s -> %s", 
                        interpolation_mode_.c_str(), new_mode.c_str());
                    interpolation_mode_ = new_mode;
                    // Reset start time so acceleration patterns start fresh
                    start_time_ = this->now();
                    
                    // ALWAYS send velocity update when switching modes!
                    // This is crucial: even if we keep position, velocity must match new mode
                    // Otherwise old vy will keep moving Y when we want it flat
                    publish_velocity_update();
                    
                    log_expected_behavior();
                }
            }
            else if (param.get_name() == "amplitude") {
                amplitude_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Amplitude changed to: %.2f", amplitude_);
                // Also update velocities when amplitude changes
                publish_velocity_update();
                log_expected_behavior();
            }
            else if (param.get_name() == "reset_on_mode_change") {
                reset_on_mode_change_ = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "reset_on_mode_change set to: %s", 
                    reset_on_mode_change_ ? "true" : "false");
            }
        }

        return result;
    }

    /**
     * @brief Get initial velocities for the current mode
     */
    void get_mode_velocities(double& vx, double& vy)
    {
        if (interpolation_mode_ == "horizontal") {
            // Horizontal line: X moves, Y stays constant
            vx = amplitude_;
            vy = 0.0;
        } 
        else if (interpolation_mode_ == "diagonal") {
            // Diagonal line: both X and Y move equally
            vx = amplitude_;
            vy = amplitude_;
        } 
        else if (interpolation_mode_ == "parabola") {
            // Parabola: X has constant velocity, Y starts at 0 and accelerates
            vx = amplitude_;
            vy = 0.0;
        }
        else {
            // All accel_ modes start from zero velocity
            vx = 0.0;
            vy = 0.0;
        }
    }

    /**
     * @brief Publish velocity update to the approximator
     * 
     * This updates the velocity in the integrator without resetting position.
     * Called when switching modes to ensure velocity matches the new mode.
     */
    void publish_velocity_update()
    {
        if (reset_pub_) {
            auto msg = odometry_interfaces_pkg::msg::PositionData();
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            
            // Get velocities for current mode
            double vx, vy;
            get_mode_velocities(vx, vy);
            msg.initial_vx = vx;
            msg.initial_vy = vy;
            
            if (reset_on_mode_change_) {
                // Full reset: position to (0,0) and set velocities
                msg.x = 0.0;
                msg.y = 0.0;
                msg.z = 0.0;
                msg.alpha = 0.0;
                reset_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), 
                    "Full reset: pos=(0,0) velocity=(%.2f, %.2f)", vx, vy);
            } else {
                // Velocity-only update: set special marker values for position
                // Use NaN to indicate "keep current position"
                msg.x = std::nan("");
                msg.y = std::nan("");
                msg.z = std::nan("");
                msg.alpha = std::nan("");
                reset_pub_->publish(msg);
                RCLCPP_INFO(this->get_logger(), 
                    "Velocity update only: vx=%.2f, vy=%.2f (position unchanged)", vx, vy);
            }
        }
    }

    /**
     * @brief Publish a full reset message to the approximator
     * 
     * This resets both position AND velocity.
     */
    void publish_reset()
    {
        if (reset_pub_) {
            auto msg = odometry_interfaces_pkg::msg::PositionData();
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.x = 0.0;
            msg.y = 0.0;
            msg.z = 0.0;
            msg.alpha = 0.0;
            
            // Get velocities for current mode
            double vx, vy;
            get_mode_velocities(vx, vy);
            msg.initial_vx = vx;
            msg.initial_vy = vy;
            
            reset_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Reset position to (0, 0, 0) with initial v=(%.2f, %.2f)",
                        msg.initial_vx, msg.initial_vy);
        }
    }

    /**
     * @brief Timer callback to publish simulated data
     * 
     * Publishes acceleration values based on the selected mode.
     */
    void publish_data()
    {
        auto current_time = this->now();
        double elapsed_sec = (current_time - start_time_).seconds();

        // Compute acceleration based on mode
        double ax = 0.0;
        double ay = 0.0;
        double omega_z = 0.0;

        // === POSITION RESULT MODES (named by what you SEE) ===
        if (interpolation_mode_ == "horizontal") {
            // Horizontal line: no acceleration, constant vx, vy=0
            ax = 0.0;
            ay = 0.0;
        } 
        else if (interpolation_mode_ == "diagonal") {
            // Diagonal line: no acceleration, constant vx=vy
            ax = 0.0;
            ay = 0.0;
        }
        else if (interpolation_mode_ == "parabola") {
            // PARABOLA: X linear (no accel), Y quadratic (constant accel)
            ax = 0.0;
            ay = amplitude_;
        }
        // === POLYNOMIAL ACCELERATION MODES (Assignment 4) ===
        else if (interpolation_mode_ == "accel_constant") {
            // Constant acceleration: a(t) = amplitude
            ax = amplitude_;
            ay = amplitude_;
        }
        else if (interpolation_mode_ == "accel_linear") {
            // Linear acceleration: a(t) = amplitude * t
            ax = amplitude_ * elapsed_sec;
            ay = amplitude_ * elapsed_sec;
        }
        else if (interpolation_mode_ == "accel_quadratic") {
            // Quadratic acceleration: a(t) = amplitude * t²
            ax = amplitude_ * elapsed_sec * elapsed_sec;
            ay = amplitude_ * elapsed_sec * elapsed_sec;
        }

        // Publish based on data type
        if (data_type_ == "acceleration") {
            // Publish custom AccelerationData message
            if (acc_pub_) {
                auto msg = odometry_interfaces_pkg::msg::AccelerationData();
                msg.header.stamp = current_time;
                msg.header.frame_id = "base_link";
                msg.linear_x = ax;
                msg.linear_y = ay;
                msg.linear_z = 0.0;
                msg.angular_z = omega_z;
                acc_pub_->publish(msg);
            }

            // Also publish as IMU message for compatibility with approximator
            if (imu_pub_) {
                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = current_time;
                imu_msg.header.frame_id = "base_link";
                imu_msg.linear_acceleration.x = ax;
                imu_msg.linear_acceleration.y = ay;
                imu_msg.linear_acceleration.z = 0.0;
                imu_msg.angular_velocity.x = 0.0;
                imu_msg.angular_velocity.y = 0.0;
                imu_msg.angular_velocity.z = omega_z;
                imu_pub_->publish(imu_msg);
            }
        }
        else if (data_type_ == "velocity" && vel_pub_) {
            // For velocity mode: compute velocity analytically
            double vx = 0.0;
            double vy = 0.0;
            
            if (interpolation_mode_ == "horizontal") {
                vx = amplitude_;
                vy = 0.0;
            }
            else if (interpolation_mode_ == "diagonal") {
                vx = amplitude_;
                vy = amplitude_;
            }
            else if (interpolation_mode_ == "parabola") {
                vx = amplitude_;
                vy = amplitude_ * elapsed_sec;
            }
            else if (interpolation_mode_ == "accel_constant") {
                vx = amplitude_ * elapsed_sec;
                vy = amplitude_ * elapsed_sec;
            }
            else if (interpolation_mode_ == "accel_linear") {
                vx = 0.5 * amplitude_ * elapsed_sec * elapsed_sec;
                vy = 0.5 * amplitude_ * elapsed_sec * elapsed_sec;
            }
            else if (interpolation_mode_ == "accel_quadratic") {
                vx = (1.0/3.0) * amplitude_ * elapsed_sec * elapsed_sec * elapsed_sec;
                vy = (1.0/3.0) * amplitude_ * elapsed_sec * elapsed_sec * elapsed_sec;
            }

            auto msg = odometry_interfaces_pkg::msg::VelocityData();
            msg.header.stamp = current_time;
            msg.header.frame_id = "base_link";
            msg.linear_x = vx;
            msg.linear_y = vy;
            msg.linear_z = 0.0;
            msg.angular_z = omega_z;
            vel_pub_->publish(msg);
        }

        // Periodic status log (every ~2 seconds at 50Hz)
        static int log_counter = 0;
        if (++log_counter >= 100) {
            RCLCPP_INFO(this->get_logger(), 
                "[t=%.1fs] Mode: %s | ax=%.3f ay=%.3f omega=%.3f",
                elapsed_sec, interpolation_mode_.c_str(), ax, ay, omega_z);
            log_counter = 0;
        }
    }

    // Member variables
    std::string data_type_;
    std::string interpolation_mode_;
    double amplitude_;
    bool reset_on_mode_change_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr startup_timer_;

    // Publishers
    rclcpp::Publisher<odometry_interfaces_pkg::msg::AccelerationData>::SharedPtr acc_pub_;
    rclcpp::Publisher<odometry_interfaces_pkg::msg::VelocityData>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<odometry_interfaces_pkg::msg::PositionData>::SharedPtr reset_pub_;

    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorDataSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
