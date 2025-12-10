"""
Launch file for IMU sensor package - MQTT/Wireless mode

This launch file starts:
1. MQTT Bridge Node - Receives IMU data from MQTT broker and publishes to ROS2 topics
2. Database Node - Subscribes to ROS2 topics and stores data in SQLite

Usage:
    ros2 launch imu_sensor_pkg imu_mqtt.launch.py

With custom MQTT broker:
    ros2 launch imu_sensor_pkg imu_mqtt.launch.py mqtt_broker:=192.168.1.100 mqtt_port:=1883
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='localhost',
        description='MQTT broker hostname or IP address'
    )
    
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )
    
    mqtt_topic_arg = DeclareLaunchArgument(
        'mqtt_topic',
        default_value='esp32/imu/data',
        description='MQTT topic to subscribe to'
    )
    
    db_path_arg = DeclareLaunchArgument(
        'db_path',
        default_value='/tmp/imu_data.db',
        description='Path to SQLite database file'
    )
    
    # MQTT Bridge Node
    mqtt_bridge_node = Node(
        package='imu_sensor_pkg',
        executable='mqtt_imu_bridge',
        name='mqtt_imu_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'mqtt_topic': LaunchConfiguration('mqtt_topic'),
        }]
    )
    
    # Database Subscriber Node
    database_node = Node(
        package='imu_sensor_pkg',
        executable='imu_database_node',
        name='imu_database_node',
        output='screen',
        parameters=[{
            # Node expects parameter 'database_path', map launch arg 'db_path' to it
            'database_path': LaunchConfiguration('db_path'),
        }]
    )
    
    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        mqtt_topic_arg,
        db_path_arg,
        mqtt_bridge_node,
        database_node,
    ])
