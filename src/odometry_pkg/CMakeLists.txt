cmake_minimum_required(VERSION 3.8)
project(odometry_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(odometry_interfaces_pkg REQUIRED)
find_package(urdf REQUIRED)

# Find SQLite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Build executables

# 1. Sensor data simulator
add_executable(sensor_data_simulator src/sensor_data_simulator.cpp)
ament_target_dependencies(sensor_data_simulator
  rclcpp
  sensor_msgs
  odometry_interfaces_pkg
)

# 2. Position and velocity approximator from acceleration
add_executable(position_velocity_approximator src/position_velocity_approximator.cpp)
ament_target_dependencies(position_velocity_approximator
  rclcpp
  sensor_msgs
  odometry_interfaces_pkg
  urdf
)

# 3. Mecanum wheel position approximator
add_executable(mecanum_position_approximator src/mecanum_position_approximator.cpp)
ament_target_dependencies(mecanum_position_approximator
  rclcpp
  std_msgs
  odometry_interfaces_pkg
)

# 4. Database handler
add_executable(database_handler src/database_handler.cpp)
ament_target_dependencies(database_handler
  rclcpp
  sensor_msgs
  odometry_interfaces_pkg
)
target_include_directories(database_handler PUBLIC ${SQLITE3_INCLUDE_DIRS})
target_link_libraries(database_handler ${SQLITE3_LIBRARIES})

# Install executables
install(TARGETS
  sensor_data_simulator
  position_velocity_approximator
  mecanum_position_approximator
  database_handler
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, rviz configs, urdf, meshes and Python scripts
install(DIRECTORY
  launch
  rviz
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/wheel_encoder_simulator.py
  scripts/position_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Find GTest for unit tests
  find_package(ament_cmake_gtest REQUIRED)

  # Test 1: Polynomial simulator tests
  ament_add_gtest(test_polynomial_simulator test/test_polynomial_simulator.cpp)
  target_include_directories(test_polynomial_simulator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Test 2: Integration tests (velocity/position approximation)
  ament_add_gtest(test_integration test/test_integration.cpp)
  target_include_directories(test_integration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Test 3: Mecanum kinematics tests
  ament_add_gtest(test_mecanum_kinematics test/test_mecanum_kinematics.cpp)
  target_include_directories(test_mecanum_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
endif()

ament_package()
