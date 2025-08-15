#include "bot_controller_demo/random_controller.hpp"
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace bot_controller_demo {

RandomController::RandomController() : Node("random_controller"), gen_(rd_()) {
  // Declare parameters with descriptions
  declare_parameter_with_description(
      "controller_type", "rviz",
      "Controller type: 'rviz' for Twist messages, 'gazebo' for TwistStamped "
      "messages");
  declare_parameter_with_description("linear_min", 0.1,
                                     "Minimum linear velocity");
  declare_parameter_with_description("linear_max", 0.2,
                                     "Maximum linear velocity");
  declare_parameter_with_description("angular_min", -1.0,
                                     "Minimum angular velocity");
  declare_parameter_with_description("angular_max", 1.0,
                                     "Maximum angular velocity");
  declare_parameter_with_description("timer_period_ms", 200,
                                     "Timer period in milliseconds");

  // Get parameters
  update_parameters();

  // Create the appropriate publisher based on controller_type parameter
  if (controller_type_ == "gazebo") {
    cmd_vel_stamped_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel",
                                                                 10);
    RCLCPP_INFO(this->get_logger(),
                "Created TwistStamped publisher for Gazebo");
  } else if (controller_type_ == "rviz") {
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(), "Created Twist publisher for RViz");
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid controller_type '%s'. Valid options are 'rviz' or "
                 "'gazebo'. Defaulting to 'rviz'.",
                 controller_type_.c_str());
    controller_type_ = "rviz";
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    RCLCPP_INFO(this->get_logger(),
                "Created Twist publisher for RViz (default)");
  }

  // Create subscriber for odometry
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&RandomController::odom_callback, this, std::placeholders::_1));

  // Create timer to publish random velocities
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_period_ms_),
      std::bind(&RandomController::timer_callback, this));

  // Initialize random distributions
  update_distributions();

  // Parameter callback for dynamic reconfiguration
  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RandomController::parameter_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Random Controller Node Started");
  RCLCPP_INFO(this->get_logger(),
              "Publishing random velocities to /cmd_vel every %d ms",
              timer_period_ms_);
  RCLCPP_INFO(this->get_logger(), "Linear velocity range: [%.2f, %.2f] m/s",
              linear_min_, linear_max_);
  RCLCPP_INFO(this->get_logger(), "Angular velocity range: [%.2f, %.2f] rad/s",
              angular_min_, angular_max_);
  RCLCPP_INFO(this->get_logger(), "Monitoring robot pose from /odom");
  RCLCPP_INFO(this->get_logger(), "Publishing %s messages",
              (controller_type_ == "gazebo") ? "TwistStamped (Gazebo)"
                                             : "Twist (RViz)");
}

void RandomController::declare_parameter_with_description(
    const std::string &name, const std::string &default_value,
    const std::string &description) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
}

void RandomController::declare_parameter_with_description(
    const std::string &name, double default_value,
    const std::string &description) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
}

void RandomController::declare_parameter_with_description(
    const std::string &name, int default_value,
    const std::string &description) {
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = description;
  this->declare_parameter(name, default_value, desc);
}

void RandomController::update_parameters() {
  controller_type_ = this->get_parameter("controller_type").as_string();
  linear_min_ = this->get_parameter("linear_min").as_double();
  linear_max_ = this->get_parameter("linear_max").as_double();
  angular_min_ = this->get_parameter("angular_min").as_double();
  angular_max_ = this->get_parameter("angular_max").as_double();
  timer_period_ms_ = this->get_parameter("timer_period_ms").as_int();
}

void RandomController::update_distributions() {
  linear_dist_ =
      std::uniform_real_distribution<double>(linear_min_, linear_max_);
  angular_dist_ =
      std::uniform_real_distribution<double>(angular_min_, angular_max_);
}

rcl_interfaces::msg::SetParametersResult RandomController::parameter_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;

  for (const auto &param : parameters) {
    if (param.get_name() == "linear_min" || param.get_name() == "linear_max" ||
        param.get_name() == "angular_min" ||
        param.get_name() == "angular_max") {

      if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        result.successful = false;
        result.reason =
            "Parameter " + param.get_name() + " must be of type double";
        return result;
      }
    } else if (param.get_name() == "timer_period_ms") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason =
            "Parameter " + param.get_name() + " must be of type integer";
        return result;
      }
    } else if (param.get_name() == "controller_type") {
      if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        result.successful = false;
        result.reason =
            "Parameter " + param.get_name() + " must be of type string";
        return result;
      }

      std::string new_type = param.as_string();
      if (new_type != "rviz" && new_type != "gazebo") {
        result.successful = false;
        result.reason =
            "Parameter controller_type must be either 'rviz' or 'gazebo'";
        return result;
      }

      RCLCPP_WARN(
          this->get_logger(),
          "Changing controller_type parameter at runtime is not supported. "
          "Restart the node to apply changes.");
    }
  }

  if (result.successful) {
    update_parameters();
    update_distributions();
    RCLCPP_INFO(this->get_logger(), "Parameters updated successfully");
  }

  return result;
}

void RandomController::publish_velocity(double linear_x, double angular_z) {
  if (controller_type_ == "gazebo") {
    // Publish TwistStamped for Gazebo
    if (cmd_vel_stamped_pub_) {
      auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
      twist_stamped_msg.header.stamp = this->get_clock()->now();
      twist_stamped_msg.header.frame_id = "base_link";
      twist_stamped_msg.twist.linear.x = linear_x;
      twist_stamped_msg.twist.linear.y = 0.0;
      twist_stamped_msg.twist.linear.z = 0.0;
      twist_stamped_msg.twist.angular.x = 0.0;
      twist_stamped_msg.twist.angular.y = 0.0;
      twist_stamped_msg.twist.angular.z = angular_z;
      cmd_vel_stamped_pub_->publish(twist_stamped_msg);
    }
  } else {
    // Publish regular Twist for RViz (default)
    if (cmd_vel_pub_) {
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = linear_x;
      twist_msg.linear.y = 0.0;
      twist_msg.linear.z = 0.0;
      twist_msg.angular.x = 0.0;
      twist_msg.angular.y = 0.0;
      twist_msg.angular.z = angular_z;
      cmd_vel_pub_->publish(twist_msg);
    }
  }
}

void RandomController::timer_callback() {
  // Generate random linear and angular velocities
  double linear_vel = linear_dist_(gen_);
  double angular_vel = angular_dist_(gen_);

  // Publish the velocity command
  publish_velocity(linear_vel, angular_vel);

  RCLCPP_DEBUG(this->get_logger(),
               "Published velocity - Linear: %.3f, Angular: %.3f", linear_vel,
               angular_vel);
}

void RandomController::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract position
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double z = msg->pose.pose.position.z;

  // Extract orientation and convert quaternion to Euler angles
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Display robot pose information
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                       1000, // Throttle to 1Hz
                       "Robot Pose - Position: [%.3f, %.3f, %.3f], "
                       "Orientation: [R=%.3f, P=%.3f, Y=%.3f] rad",
                       x, y, z, roll, pitch, yaw);

  // Also display in degrees for easier understanding
  RCLCPP_DEBUG(this->get_logger(),
               "Orientation in degrees: [R=%.1f°, P=%.1f°, Y=%.1f°]",
               roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
}

} // namespace bot_controller_demo

// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bot_controller_demo::RandomController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}