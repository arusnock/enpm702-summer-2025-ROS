#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bot_controller_demo {

class RandomController : public rclcpp::Node {
public:
  RandomController();

private:
  void declare_parameter_with_description(const std::string &name,
                                          const std::string &default_value,
                                          const std::string &description);
  void declare_parameter_with_description(const std::string &name,
                                          double default_value,
                                          const std::string &description);
  void declare_parameter_with_description(const std::string &name,
                                          int default_value,
                                          const std::string &description);

  void update_parameters();
  void update_distributions();
  void publish_velocity(double linear_x, double angular_z);
  void timer_callback();
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult
  parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

  // Member variables
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
      cmd_vel_stamped_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  // Parameters
  std::string controller_type_;
  double linear_min_;
  double linear_max_;
  double angular_min_;
  double angular_max_;
  int timer_period_ms_;

  // Random number generation
  std::random_device rd_;
  std::mt19937 gen_;
  std::uniform_real_distribution<double> linear_dist_;
  std::uniform_real_distribution<double> angular_dist_;
};

} // namespace bot_controller_demo
