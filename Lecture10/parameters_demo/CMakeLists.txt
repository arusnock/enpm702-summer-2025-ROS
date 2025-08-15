cmake_minimum_required(VERSION 3.8)
project(parameters_demo)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Include directories
include_directories(include)

# Create executables
add_executable(camera_demo
  src/camera_demo.cpp
)

add_executable(lidar_demo
  src/lidar_demo.cpp
)

add_executable(radar_demo
  src/radar_demo.cpp
)

add_executable(processing_demo
  src/processing_demo.cpp
)

# Specify dependencies for the executable
ament_target_dependencies(camera_demo
  rclcpp
  sensor_msgs
  rcl_interfaces
)

ament_target_dependencies(lidar_demo
  rclcpp
  sensor_msgs
  rcl_interfaces
)

ament_target_dependencies(radar_demo
  rclcpp
  radar_msgs
  rcl_interfaces
)

ament_target_dependencies(processing_demo
  rclcpp
  sensor_msgs
  radar_msgs
  rcl_interfaces
)


# Install executable
install(TARGETS
  camera_demo
  lidar_demo
  radar_demo
  processing_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if any)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  # PATTERN "*.launch.xml"
)

# Install config files (if any)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Lint tests
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  rcl_interfaces
  radar_msgs
)

# Ament package
ament_package()