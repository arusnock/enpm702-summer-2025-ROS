#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <radar_msgs/msg/radar_scan.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/qos.hpp>
#include "parameters_demo/color_utils.hpp"
#include <memory>
#include <string>
#include <vector>

namespace parameters_demo_nodes
{

class ProcessingDemoNode : public rclcpp::Node
{
public:
  ProcessingDemoNode();

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void radar_callback(const radar_msgs::msg::RadarScan::SharedPtr msg);
  void process_data_callback();
  void print_processing_status();

  // Parameters
  std::string processing_mode_;
  int processing_rate_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr sensor_cb_group_;
  rclcpp::CallbackGroup::SharedPtr processing_cb_group_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
  rclcpp::Subscription<radar_msgs::msg::RadarScan>::SharedPtr radar_subscriber_;

  // Timer for processing
  rclcpp::TimerBase::SharedPtr processing_timer_;

  // Data storage for sensor readings
  sensor_msgs::msg::Image::SharedPtr latest_camera_data_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_lidar_data_;
  radar_msgs::msg::RadarScan::SharedPtr latest_radar_data_;

  // Statistics counters
  uint64_t camera_frames_received_;
  uint64_t lidar_scans_received_;
  uint64_t radar_scans_received_;
  uint64_t processing_cycles_;
};

}  // namespace parameters_demo_nodes