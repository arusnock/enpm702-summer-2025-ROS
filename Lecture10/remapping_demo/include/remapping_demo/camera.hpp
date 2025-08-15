#pragma once

#include <cstdint>
#include <memory>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>

namespace remapping_demo {

class CameraDemoNode : public rclcpp::Node {
public:
  CameraDemoNode();

private:
  void data_camera_pub_callback();

  rcl_interfaces::msg::SetParametersResult
  parameter_update_callback(const std::vector<rclcpp::Parameter> &params);

  void generate_image_data();

  // Parameters
  std::string camera_name_;
  std::string camera_frame_;
  int camera_rate_;

  // Image properties
  int image_width_;
  int image_height_;
  int image_channels_;
  int image_step_;
  std::vector<uint8_t> image_data_;

  // ROS components
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr data_camera_publisher_;
  rclcpp::TimerBase::SharedPtr data_camera_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_callback_handle_;

  // Message
  sensor_msgs::msg::Image data_camera_msg_;
  int frame_counter_;
};

} // namespace remapping_demo