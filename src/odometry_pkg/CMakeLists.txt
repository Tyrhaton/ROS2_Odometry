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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find SQLite3
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

# Build executables

# 1. Position and velocity approximator from acceleration
add_executable(position_velocity_approximator src/position_velocity_approximator.cpp)
ament_target_dependencies(position_velocity_approximator
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  urdf
)

# 3. Mecanum wheel position approximator
add_executable(mecanum_position_approximator src/mecanum_position_approximator.cpp)
ament_target_dependencies(mecanum_position_approximator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# 4. Database handler
add_executable(database_handler src/database_handler.cpp)
ament_target_dependencies(database_handler
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)
target_include_directories(database_handler PUBLIC ${SQLITE3_INCLUDE_DIRS})
target_link_libraries(database_handler ${SQLITE3_LIBRARIES})

# 5a. IMU Simulator (C++)
add_executable(imu_simulator src/imu_simulator.cpp)
ament_target_dependencies(imu_simulator
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  ament_index_cpp
)
target_link_libraries(imu_simulator yaml-cpp)

# 5b. Mecanum Simulator (C++)
add_executable(mecanum_simulator src/mecanum_simulator.cpp)
ament_target_dependencies(mecanum_simulator
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  ament_index_cpp
)
target_link_libraries(mecanum_simulator yaml-cpp)

# 6. Position Visualizer (C++)
add_executable(position_visualizer src/position_visualizer.cpp)
ament_target_dependencies(position_visualizer
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  ament_index_cpp
)

# Install executables
install(TARGETS
  position_velocity_approximator
  mecanum_position_approximator
  database_handler
  imu_simulator
  mecanum_simulator
  position_visualizer
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

# No Python scripts - pure C++ package

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
