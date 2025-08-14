#pragma once

#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <radar_msgs/msg/radar_return.hpp>
#include <std_msgs/msg/header.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>
#include <string>
#include <vector>

namespace parameters_demo_nodes
{

class RadarScanDemoNode : public rclcpp::Node
{
public:
  RadarScanDemoNode();

private:
  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & params);

  void radar_tracks_pub_callback();

  // Parameters
  std::string radar_name_;
  int radar_rate_;

  // ROS2 components
  rclcpp::Publisher<radar_msgs::msg::RadarScan>::SharedPtr radar_tracks_publisher_;
  rclcpp::TimerBase::SharedPtr radar_tracks_timer_;
  radar_msgs::msg::RadarScan radar_msg_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace parameters_demo_nodes