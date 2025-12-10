/**
 * @file database_handler.cpp
 * @brief Database handler node for storing sensor and position data
 *
 * This node subscribes to configurable topics and stores the received data
 * in a SQLite database for later analysis and testing.
 *
 * @author Group g1
 * @date 2025
 */

#include <chrono>
#include <memory>
#include <string>
#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "odometry_interfaces_pkg/msg/acceleration_data.hpp"
#include "odometry_interfaces_pkg/msg/velocity_data.hpp"
#include "odometry_interfaces_pkg/msg/position_data.hpp"

using namespace std::chrono_literals;

/**
 * @class DatabaseHandler
 * @brief Subscribes to topics and stores data in SQLite database
 */
class DatabaseHandler : public rclcpp::Node
{
public:
    DatabaseHandler() : Node("database_handler"), db_(nullptr)
    {
        // Declare parameters
        this->declare_parameter<std::string>("database_path", "odometry_data.db");
        this->declare_parameter<bool>("subscribe_imu", true);
        this->declare_parameter<bool>("subscribe_acceleration", false);
        this->declare_parameter<bool>("subscribe_velocity", true);
        this->declare_parameter<bool>("subscribe_position", true);

        // Get database path
        db_path_ = this->get_parameter("database_path").as_string();

        // Open database
        if (!open_database()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open database, node will not function");
            return;
        }

        // Create tables
        if (!create_tables()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create database tables");
            close_database();
            return;
        }

        // Create subscriptions based on parameters
        if (this->get_parameter("subscribe_imu").as_bool()) {
            // Use sensor QoS to match IMU publishers (best effort, small history).
            auto sensor_qos = rclcpp::SensorDataQoS();
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/imu/data",
                sensor_qos,
                std::bind(&DatabaseHandler::imu_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /imu/data");
        }

        if (this->get_parameter("subscribe_acceleration").as_bool()) {
            accel_sub_ = this->create_subscription<odometry_interfaces_pkg::msg::AccelerationData>(
                "/simulator/acceleration",
                10,
                std::bind(&DatabaseHandler::acceleration_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /simulator/acceleration");
        }

        if (this->get_parameter("subscribe_velocity").as_bool()) {
            vel_sub_ = this->create_subscription<odometry_interfaces_pkg::msg::VelocityData>(
                "/odometry/velocity_from_accel",
                10,
                std::bind(&DatabaseHandler::velocity_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /odometry/velocity_from_accel");
        }

        if (this->get_parameter("subscribe_position").as_bool()) {
            pos_sub_ = this->create_subscription<odometry_interfaces_pkg::msg::PositionData>(
                "/odometry/position_from_accel",
                10,
                std::bind(&DatabaseHandler::position_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /odometry/position_from_accel");
        }

        RCLCPP_INFO(this->get_logger(), "Database Handler started with database: %s", db_path_.c_str());
    }

    ~DatabaseHandler()
    {
        close_database();
    }

private:
    /**
     * @brief Open SQLite database connection
     * @return true if successful
     */
    bool open_database()
    {
        int rc = sqlite3_open(db_path_.c_str(), &db_);
        if (rc != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open database: %s", sqlite3_errmsg(db_));
            return false;
        }
        return true;
    }

    /**
     * @brief Close database connection
     */
    void close_database()
    {
        if (db_) {
            sqlite3_close(db_);
            db_ = nullptr;
        }
    }

    /**
     * @brief Create database tables if they don't exist
     * @return true if successful
     */
    bool create_tables()
    {
        const char* sql_imu = R"(
            CREATE TABLE IF NOT EXISTS imu_measurements (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_sec INTEGER,
                timestamp_nsec INTEGER,
                linear_accel_x REAL,
                linear_accel_y REAL,
                linear_accel_z REAL,
                angular_vel_x REAL,
                angular_vel_y REAL,
                angular_vel_z REAL
            );
        )";

        const char* sql_velocity = R"(
            CREATE TABLE IF NOT EXISTS velocity_measurements (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_sec INTEGER,
                timestamp_nsec INTEGER,
                linear_x REAL,
                linear_y REAL,
                linear_z REAL,
                angular_z REAL
            );
        )";

        const char* sql_position = R"(
            CREATE TABLE IF NOT EXISTS position_measurements (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp_sec INTEGER,
                timestamp_nsec INTEGER,
                x REAL,
                y REAL,
                z REAL,
                alpha REAL
            );
        )";

        char* err_msg = nullptr;

        // Create IMU table
        if (sqlite3_exec(db_, sql_imu, nullptr, nullptr, &err_msg) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", err_msg);
            sqlite3_free(err_msg);
            return false;
        }

        // Create velocity table
        if (sqlite3_exec(db_, sql_velocity, nullptr, nullptr, &err_msg) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", err_msg);
            sqlite3_free(err_msg);
            return false;
        }

        // Create position table
        if (sqlite3_exec(db_, sql_position, nullptr, nullptr, &err_msg) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "SQL error: %s", err_msg);
            sqlite3_free(err_msg);
            return false;
        }

        return true;
    }

    /**
     * @brief Callback for IMU data
     */
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const char* sql = R"(
            INSERT INTO imu_measurements
            (timestamp_sec, timestamp_nsec, linear_accel_x, linear_accel_y, linear_accel_z,
             angular_vel_x, angular_vel_y, angular_vel_z)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?);
        )";

        sqlite3_stmt* stmt;
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        sqlite3_bind_int64(stmt, 1, msg->header.stamp.sec);
        sqlite3_bind_int64(stmt, 2, msg->header.stamp.nanosec);
        sqlite3_bind_double(stmt, 3, msg->linear_acceleration.x);
        sqlite3_bind_double(stmt, 4, msg->linear_acceleration.y);
        sqlite3_bind_double(stmt, 5, msg->linear_acceleration.z);
        sqlite3_bind_double(stmt, 6, msg->angular_velocity.x);
        sqlite3_bind_double(stmt, 7, msg->angular_velocity.y);
        sqlite3_bind_double(stmt, 8, msg->angular_velocity.z);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert IMU data: %s", sqlite3_errmsg(db_));
        }

        sqlite3_finalize(stmt);
    }

    /**
     * @brief Callback for acceleration data
     */
    void acceleration_callback(const odometry_interfaces_pkg::msg::AccelerationData::SharedPtr msg)
    {
        // Similar to IMU, but with custom message format
        // For simplicity, we can reuse the IMU table or create a separate one
        RCLCPP_DEBUG(this->get_logger(), "Received acceleration data at t=%.3f",
                     msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    }

    /**
     * @brief Callback for velocity data
     */
    void velocity_callback(const odometry_interfaces_pkg::msg::VelocityData::SharedPtr msg)
    {
        const char* sql = R"(
            INSERT INTO velocity_measurements
            (timestamp_sec, timestamp_nsec, linear_x, linear_y, linear_z, angular_z)
            VALUES (?, ?, ?, ?, ?, ?);
        )";

        sqlite3_stmt* stmt;
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        sqlite3_bind_int64(stmt, 1, msg->header.stamp.sec);
        sqlite3_bind_int64(stmt, 2, msg->header.stamp.nanosec);
        sqlite3_bind_double(stmt, 3, msg->linear_x);
        sqlite3_bind_double(stmt, 4, msg->linear_y);
        sqlite3_bind_double(stmt, 5, msg->linear_z);
        sqlite3_bind_double(stmt, 6, msg->angular_z);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert velocity data: %s", sqlite3_errmsg(db_));
        }

        sqlite3_finalize(stmt);
    }

    /**
     * @brief Callback for position data
     */
    void position_callback(const odometry_interfaces_pkg::msg::PositionData::SharedPtr msg)
    {
        const char* sql = R"(
            INSERT INTO position_measurements
            (timestamp_sec, timestamp_nsec, x, y, z, alpha)
            VALUES (?, ?, ?, ?, ?, ?);
        )";

        sqlite3_stmt* stmt;
        if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK) {
            RCLCPP_ERROR(this->get_logger(), "Failed to prepare statement: %s", sqlite3_errmsg(db_));
            return;
        }

        sqlite3_bind_int64(stmt, 1, msg->header.stamp.sec);
        sqlite3_bind_int64(stmt, 2, msg->header.stamp.nanosec);
        sqlite3_bind_double(stmt, 3, msg->x);
        sqlite3_bind_double(stmt, 4, msg->y);
        sqlite3_bind_double(stmt, 5, msg->z);
        sqlite3_bind_double(stmt, 6, msg->alpha);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert position data: %s", sqlite3_errmsg(db_));
        }

        sqlite3_finalize(stmt);
    }

    // Member variables
    std::string db_path_;
    sqlite3* db_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<odometry_interfaces_pkg::msg::AccelerationData>::SharedPtr accel_sub_;
    rclcpp::Subscription<odometry_interfaces_pkg::msg::VelocityData>::SharedPtr vel_sub_;
    rclcpp::Subscription<odometry_interfaces_pkg::msg::PositionData>::SharedPtr pos_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatabaseHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
