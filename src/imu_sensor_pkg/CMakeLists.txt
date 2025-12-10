cmake_minimum_required(VERSION 3.8)
project(imu_sensor_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(include)

# Serial Lifecycle Node
add_executable(serial_imu_node src/serial_imu_lifecycle_node.cpp)
ament_target_dependencies(serial_imu_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  lifecycle_msgs
)

# Database Subscriber Node
add_executable(imu_database_node src/imu_database_subscriber.cpp)
ament_target_dependencies(imu_database_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(imu_database_node sqlite3)

# MQTT Bridge Node
add_executable(mqtt_imu_bridge src/mqtt_imu_bridge.cpp)
ament_target_dependencies(mqtt_imu_bridge
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(mqtt_imu_bridge mosquitto)

# Install executables
install(TARGETS
  serial_imu_node
  imu_database_node
  mqtt_imu_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
