#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <mosquitto.h>
#include <string>
#include <sstream>
#include <vector>

class MQTTIMUBridge : public rclcpp::Node
{
public:
    MQTTIMUBridge() : Node("mqtt_imu_bridge")
    {
        // Declare parameters
        this->declare_parameter("mqtt_broker", "localhost");
        this->declare_parameter("mqtt_port", 1883);
        this->declare_parameter("mqtt_topic", "esp32/imu/data");
        
        mqtt_broker_ = this->get_parameter("mqtt_broker").as_string();
        mqtt_port_ = this->get_parameter("mqtt_port").as_int();
        mqtt_topic_ = this->get_parameter("mqtt_topic").as_string();
        
        // Create ROS2 publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
        
        // Initialize mosquitto library
        mosquitto_lib_init();
        
        // Create mosquitto instance
        mosq_ = mosquitto_new(nullptr, true, this);
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mosquitto instance");
            return;
        }
        
        // Set callbacks
        mosquitto_connect_callback_set(mosq_, on_connect_wrapper);
        mosquitto_message_callback_set(mosq_, on_message_wrapper);
        
        // Connect to broker
        int rc = mosquitto_connect(mosq_, mqtt_broker_.c_str(), mqtt_port_, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", mosquitto_strerror(rc));
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to MQTT broker %s:%d", mqtt_broker_.c_str(), mqtt_port_);
        
        // Start mosquitto loop in separate thread
        mosquitto_loop_start(mosq_);
    }
    
    ~MQTTIMUBridge()
    {
        if (mosq_) {
            mosquitto_loop_stop(mosq_, true);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

private:
    static void on_connect_wrapper(struct mosquitto *mosq, void *obj, int rc)
    {
        auto *node = static_cast<MQTTIMUBridge*>(obj);
        node->on_connect(mosq, rc);
    }
    
    static void on_message_wrapper(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg)
    {
        auto *node = static_cast<MQTTIMUBridge*>(obj);
        node->on_message(mosq, msg);
    }
    
    void on_connect(struct mosquitto *mosq, int rc)
    {
        if (rc == 0) {
            RCLCPP_INFO(this->get_logger(), "MQTT connected successfully");
            // Subscribe to topic
            mosquitto_subscribe(mosq, nullptr, mqtt_topic_.c_str(), 0);
            RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", mqtt_topic_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "MQTT connection failed with code %d", rc);
        }
    }
    
    void on_message(struct mosquitto *mosq, const struct mosquitto_message *msg)
    {
        (void)mosq; // Unused
        
        std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
        RCLCPP_DEBUG(this->get_logger(), "Received MQTT message: %s", payload.c_str());
        
        // Parse CSV: IMU,timestamp,ax,ay,az,gx,gy,gz,temp
        std::vector<std::string> tokens;
        std::stringstream ss(payload);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }
        
        if (tokens.size() != 9 || tokens[0] != "IMU") {
            RCLCPP_WARN(this->get_logger(), "Invalid CSV format: %s", payload.c_str());
            return;
        }
        
        try {
            // Parse values (skip tokens[1] timestamp_ms, use ROS2 time instead)
            double ax = std::stod(tokens[2]);
            double ay = std::stod(tokens[3]);
            double az = std::stod(tokens[4]);
            double gx = std::stod(tokens[5]);
            double gy = std::stod(tokens[6]);
            double gz = std::stod(tokens[7]);
            double temp = std::stod(tokens[8]);
            
            // Create IMU message
            auto imu_msg = sensor_msgs::msg::Imu();

            // Parse and apply sensor-provided timestamp (milliseconds) when possible
            try {
                uint64_t timestamp_ms = std::stoull(tokens[1]);
                uint64_t sec = timestamp_ms / 1000u;
                uint64_t nsec = (timestamp_ms % 1000u) * 1000000u;
                imu_msg.header.stamp.sec = static_cast<int32_t>(sec);
                imu_msg.header.stamp.nanosec = static_cast<uint32_t>(nsec);
            } catch (const std::exception &e) {
                // Fallback to node time
                imu_msg.header.stamp = this->now();
            }
            imu_msg.header.frame_id = "imu_link";
            
            imu_msg.linear_acceleration.x = ax;
            imu_msg.linear_acceleration.y = ay;
            imu_msg.linear_acceleration.z = az;
            
            imu_msg.angular_velocity.x = gx;
            imu_msg.angular_velocity.y = gy;
            imu_msg.angular_velocity.z = gz;
            
            // Publish IMU data
            imu_pub_->publish(imu_msg);
            
            // Create temperature message
            auto temp_msg = sensor_msgs::msg::Temperature();
            temp_msg.header.stamp = imu_msg.header.stamp;
            temp_msg.header.frame_id = "imu_link";
            temp_msg.temperature = temp;
            
            // Publish temperature
            temp_pub_->publish(temp_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Published IMU data: ax=%.3f, ay=%.3f, az=%.3f", ax, ay, az);
            
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse CSV: %s", e.what());
        }
    }
    
    std::string mqtt_broker_;
    int mqtt_port_;
    std::string mqtt_topic_;
    
    struct mosquitto *mosq_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MQTTIMUBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
