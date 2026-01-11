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
#include <fstream>
#include <iomanip>
#include <sqlite3.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
        
        // CSV export parameters
        this->declare_parameter<bool>("enable_csv_export", true);
        this->declare_parameter<std::string>("csv_output_dir", "csvexport");
        this->declare_parameter<int>("csv_export_interval_ms", 10);  // Export every 100ms by default

        // Get database path
        db_path_ = this->get_parameter("database_path").as_string();
        
        // Get CSV export parameters
        enable_csv_export_ = this->get_parameter("enable_csv_export").as_bool();
        csv_output_dir_ = this->get_parameter("csv_output_dir").as_string();
        csv_export_interval_ms_ = this->get_parameter("csv_export_interval_ms").as_int();

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
        
        // Initialize CSV export if enabled
        if (enable_csv_export_) {
            if (!init_csv_export()) {
                RCLCPP_WARN(this->get_logger(), "Failed to initialize CSV export");
                enable_csv_export_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "CSV export enabled: directory=%s, interval=%dms", 
                           csv_output_dir_.c_str(), csv_export_interval_ms_);
                
                // Record start time for relative timestamps
                start_time_ = this->now();
            }
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
            accel_sub_ = this->create_subscription<geometry_msgs::msg::AccelStamped>(
                "/simulator/acceleration",
                10,
                std::bind(&DatabaseHandler::acceleration_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /simulator/acceleration");
        }

        if (this->get_parameter("subscribe_velocity").as_bool()) {
            vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "/odometry/velocity_from_accel",
                10,
                std::bind(&DatabaseHandler::velocity_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /odometry/velocity_from_accel");
        }

        if (this->get_parameter("subscribe_position").as_bool()) {
            pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odometry/position_from_accel",
                10,
                std::bind(&DatabaseHandler::position_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscribed to /odometry/position_from_accel");
        }

        RCLCPP_INFO(this->get_logger(), "Database Handler started with database: %s", db_path_.c_str());
    }

    ~DatabaseHandler()
    {
        close_csv_files();
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
     * @brief Initialize CSV export files
     * @return true if successful
     */
    bool init_csv_export()
    {
        // Create output directory if it doesn't exist
        std::string mkdir_cmd = "mkdir -p " + csv_output_dir_;
        if (system(mkdir_cmd.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CSV output directory: %s", csv_output_dir_.c_str());
            return false;
        }

        // Generate timestamp for unique filenames
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");
        std::string timestamp = ss.str();

        // Open CSV files
        std::string imu_path = csv_output_dir_ + "/imu_data_" + timestamp + ".csv";
        std::string vel_path = csv_output_dir_ + "/velocity_data_" + timestamp + ".csv";
        std::string pos_path = csv_output_dir_ + "/position_data_" + timestamp + ".csv";

        imu_csv_.open(imu_path);
        velocity_csv_.open(vel_path);
        position_csv_.open(pos_path);

        if (!imu_csv_.is_open() || !velocity_csv_.is_open() || !position_csv_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV files");
            return false;
        }

        // Write CSV headers
        imu_csv_ << "time,linear_accel_x,linear_accel_y,linear_accel_z,angular_vel_x,angular_vel_y,angular_vel_z\n";
        velocity_csv_ << "time,linear_x,linear_y,linear_z,angular_z\n";
        position_csv_ << "time,x,y,z,alpha\n";

        imu_csv_ << std::fixed << std::setprecision(6);
        velocity_csv_ << std::fixed << std::setprecision(6);
        position_csv_ << std::fixed << std::setprecision(6);

        RCLCPP_INFO(this->get_logger(), "CSV files created:");
        RCLCPP_INFO(this->get_logger(), "  IMU: %s", imu_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  Velocity: %s", vel_path.c_str());
        RCLCPP_INFO(this->get_logger(), "  Position: %s", pos_path.c_str());

        return true;
    }

    /**
     * @brief Close CSV files
     */
    void close_csv_files()
    {
        if (imu_csv_.is_open()) {
            imu_csv_.close();
        }
        if (velocity_csv_.is_open()) {
            velocity_csv_.close();
        }
        if (position_csv_.is_open()) {
            position_csv_.close();
        }
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
        
        // CSV export - write EVERY message without filtering
        if (enable_csv_export_ && imu_csv_.is_open()) {
            rclcpp::Time msg_time(msg->header.stamp);
            
            // Use message timestamp directly (simulator uses sim_time starting from 0)
            double relative_time = msg_time.seconds();
            imu_csv_ << relative_time << ","
                    << msg->linear_acceleration.x << "," 
                    << msg->linear_acceleration.y << "," 
                    << msg->linear_acceleration.z << ","
                    << msg->angular_velocity.x << "," 
                    << msg->angular_velocity.y << "," 
                    << msg->angular_velocity.z << "\n";
            imu_csv_.flush();
        }
    }

    /**
     * @brief Callback for acceleration data
     */
    void acceleration_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg)
    {
        // Similar to IMU, but with AccelStamped message format
        RCLCPP_DEBUG(this->get_logger(), "Received acceleration data at t=%.3f",
                     msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    }

    /**
     * @brief Callback for velocity data
     */
    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
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
        sqlite3_bind_double(stmt, 3, msg->twist.linear.x);
        sqlite3_bind_double(stmt, 4, msg->twist.linear.y);
        sqlite3_bind_double(stmt, 5, msg->twist.linear.z);
        sqlite3_bind_double(stmt, 6, msg->twist.angular.z);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert velocity data: %s", sqlite3_errmsg(db_));
        }

        sqlite3_finalize(stmt);
        
        // CSV export - write EVERY message without filtering
        if (enable_csv_export_ && velocity_csv_.is_open()) {
            rclcpp::Time msg_time(msg->header.stamp);
            
            // Use message timestamp directly (simulator uses sim_time starting from 0)
            double relative_time = msg_time.seconds();
            velocity_csv_ << relative_time << ","
                        << msg->twist.linear.x << "," 
                        << msg->twist.linear.y << "," 
                        << msg->twist.linear.z << "," 
                        << msg->twist.angular.z << "\n";
            velocity_csv_.flush();
        }
    }

    /**
     * @brief Callback for position data
     */
    void position_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

        // Extract yaw from quaternion
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        double alpha = 2.0 * std::atan2(qz, qw);

        sqlite3_bind_int64(stmt, 1, msg->header.stamp.sec);
        sqlite3_bind_int64(stmt, 2, msg->header.stamp.nanosec);
        sqlite3_bind_double(stmt, 3, msg->pose.pose.position.x);
        sqlite3_bind_double(stmt, 4, msg->pose.pose.position.y);
        sqlite3_bind_double(stmt, 5, msg->pose.pose.position.z);
        sqlite3_bind_double(stmt, 6, alpha);

        if (sqlite3_step(stmt) != SQLITE_DONE) {
            RCLCPP_ERROR(this->get_logger(), "Failed to insert position data: %s", sqlite3_errmsg(db_));
        }

        sqlite3_finalize(stmt);
        
        // CSV export - write EVERY message without filtering
        if (enable_csv_export_ && position_csv_.is_open()) {
            rclcpp::Time msg_time(msg->header.stamp);
            
            // Use message timestamp directly (simulator uses sim_time starting from 0)
            double relative_time = msg_time.seconds();
            position_csv_ << relative_time << ","
                        << msg->pose.pose.position.x << "," 
                        << msg->pose.pose.position.y << "," 
                        << msg->pose.pose.position.z << "," 
                        << alpha << "\n";
            position_csv_.flush();
        }
    }

    // Member variables
    std::string db_path_;
    sqlite3* db_;

    // CSV export
    bool enable_csv_export_{false};
    std::string csv_output_dir_;
    int csv_export_interval_ms_{10};
    std::ofstream imu_csv_;
    std::ofstream velocity_csv_;
    std::ofstream position_csv_;
    rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_imu_export_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_vel_export_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_pos_export_time_{0, 0, RCL_ROS_TIME};

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DatabaseHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
