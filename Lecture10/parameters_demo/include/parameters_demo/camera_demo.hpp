#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "parameters_demo/color_utils.hpp"
#include <memory>
#include <string>
#include <vector>

namespace parameters_demo_nodes
{

class CameraDemoNode : public rclcpp::Node
{
public:
  CameraDemoNode();

private:
  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & params);

  void data_camera_pub_callback();

  void generate_random_image();

  // Parameters
  std::string camera_name_;
  int camera_rate_;

  // Image properties
  static constexpr int image_width_ = 640;
  static constexpr int image_height_ = 480;
  static constexpr int image_channels_ = 3;
  static constexpr int image_step_ = image_width_ * image_channels_;
  
  std::vector<uint8_t> image_data_;
  
  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr data_camera_publisher_;
  rclcpp::TimerBase::SharedPtr data_camera_timer_;
  sensor_msgs::msg::Image data_camera_msg_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // Frame counter for logging
  uint64_t frame_counter_;
};

}  // namespace parameters_demo_nodes