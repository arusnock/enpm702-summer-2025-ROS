cmake_minimum_required(VERSION 3.8)
project(remapping_demo)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Create executables
add_executable(camera_demo
  src/camera.cpp
)

# Specify dependencies for the executable
ament_target_dependencies(camera_demo
  rclcpp
  sensor_msgs
)


# Install executable
install(TARGETS
  camera_demo
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
)

# Ament package
ament_package()