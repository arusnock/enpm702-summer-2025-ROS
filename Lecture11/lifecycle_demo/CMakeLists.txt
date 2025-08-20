cmake_minimum_required(VERSION 3.8)
project(lifecycle_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

# Add include directories
include_directories(include)

# Sensor Publisher Lifecycle Node
add_executable(sensor_publisher src/sensor_publisher.cpp)
ament_target_dependencies(sensor_publisher
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# Install executables
install(TARGETS
  sensor_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()