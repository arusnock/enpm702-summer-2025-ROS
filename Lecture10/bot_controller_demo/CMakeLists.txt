cmake_minimum_required(VERSION 3.8)
project(bot_controller_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Add include directories
include_directories(include)

# Create executable
add_executable(random_controller src/random_controller.cpp)
add_executable(proportional_controller src/proportional_controller.cpp)

# Specify dependencies
ament_target_dependencies(random_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_target_dependencies(proportional_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  rcl_interfaces
)

# Install targets
install(TARGETS
  random_controller
  proportional_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()