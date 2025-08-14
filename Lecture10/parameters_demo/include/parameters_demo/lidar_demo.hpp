#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <memory>
#include <string>
#include <vector>

namespace parameters_demo_nodes
{

class LidarDemoNode : public rclcpp::Node
{
public:
  LidarDemoNode();

private:
  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & params);

  void data_lidar_pub_callback();

  // Parameters
  std::string lidar_name_;
  std::string lidar_model_;
  int lidar_rate_;

  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr data_lidar_publisher_;
  rclcpp::TimerBase::SharedPtr data_lidar_timer_;
  sensor_msgs::msg::LaserScan data_lidar_msg_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace parameters_demo_nodes