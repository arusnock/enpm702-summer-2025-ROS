#include "parameters_demo/radar_demo.hpp"
#include <random>
#include <chrono>
#include <cmath>

namespace parameters_demo_nodes
{

RadarScanDemoNode::RadarScanDemoNode()
: Node("radar_demo"), 
  radar_name_("radar"),
  radar_rate_(79)
{
  // Declare all parameters for the node
  
  // Declare radar_name parameter
  rcl_interfaces::msg::ParameterDescriptor radar_name_desc;
  radar_name_desc.description = "Radar name";
  this->declare_parameter("radar_name", "radar", radar_name_desc);

  // Declare radar_rate parameter with integer range constraint
  rcl_interfaces::msg::ParameterDescriptor radar_rate_desc;
  radar_rate_desc.description = "Radar frame rate in Hz";
  rcl_interfaces::msg::IntegerRange rate_range;
  rate_range.from_value = 55;
  rate_range.to_value = 80;
  rate_range.step = 1;
  radar_rate_desc.integer_range.push_back(rate_range);
  this->declare_parameter("radar_rate", 79, radar_rate_desc);

  // Get Parameters
  radar_name_ = this->get_parameter("radar_name").as_string();
  radar_rate_ = this->get_parameter("radar_rate").as_int();

  // Register parameter callback for dynamic parameter updates
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RadarScanDemoNode::parameter_update_callback, this, std::placeholders::_1));

  // Set up publisher for radar scans
  radar_tracks_publisher_ = this->create_publisher<radar_msgs::msg::RadarScan>(
    "/radar/tracks", 10);

  // Set up timer for periodic publishing
  auto timer_period = std::chrono::duration<double>(1.0 / radar_rate_);
  radar_tracks_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&RadarScanDemoNode::radar_tracks_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "RadarScan publisher node initialized with name: %s, rate: %d Hz", 
              radar_name_.c_str(), radar_rate_);
}

rcl_interfaces::msg::SetParametersResult RadarScanDemoNode::parameter_update_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : params) {
    if (param.get_name() == "radar_name") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        radar_name_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Updated radar_name to: %s", radar_name_.c_str());
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for radar_name";
        return result;
      }
    } else if (param.get_name() == "radar_rate") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int new_rate = param.as_int();
        if (new_rate >= 55 && new_rate <= 80) {
          radar_rate_ = new_rate;
          RCLCPP_INFO(this->get_logger(), "Updated radar_rate to: %d", radar_rate_);
          
          // Update timer with new rate
          if (radar_tracks_timer_) {
            radar_tracks_timer_.reset();
            auto timer_period = std::chrono::duration<double>(1.0 / radar_rate_);
            radar_tracks_timer_ = this->create_wall_timer(
              std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
              std::bind(&RadarScanDemoNode::radar_tracks_pub_callback, this));
            RCLCPP_INFO(this->get_logger(), "Timer rate updated to: %d Hz", radar_rate_);
          }
        } else {
          result.successful = false;
          result.reason = "radar_rate must be between 55 and 80 Hz";
          return result;
        }
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for radar_rate";
        return result;
      }
    }
  }

  return result;
}

void RadarScanDemoNode::radar_tracks_pub_callback()
{
  // Set header with current time
  radar_msg_.header.stamp = this->get_clock()->now();
  radar_msg_.header.frame_id = "radar_link";

  // Initialize random number generators
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<int> num_returns_dis(1, 10);
  static std::uniform_real_distribution<double> range_dis(0.0, 100.0);
  static std::uniform_real_distribution<double> azimuth_dis(-M_PI / 2.0, M_PI / 2.0);
  static std::uniform_real_distribution<double> elevation_dis(-M_PI / 6.0, M_PI / 6.0);
  static std::uniform_real_distribution<double> doppler_dis(-50.0, 50.0);
  static std::uniform_real_distribution<double> amplitude_dis(0.0, 100.0);

  // Random number of returns (1-10)
  int num_returns = num_returns_dis(gen);

  // Clear previous returns
  radar_msg_.returns.clear();
  radar_msg_.returns.reserve(num_returns);

  // Generate random returns
  for (int i = 0; i < num_returns; ++i) {
    radar_msgs::msg::RadarReturn radar_return;

    // Random range in meters (0-100m)
    radar_return.range = range_dis(gen);

    // Random azimuth angle in radians (-π/2 to π/2)
    radar_return.azimuth = azimuth_dis(gen);

    // Random elevation angle in radians (-π/6 to π/6)
    radar_return.elevation = elevation_dis(gen);

    // Random Doppler velocity between -50 and 50 m/s
    radar_return.doppler_velocity = doppler_dis(gen);

    // Random amplitude between 0 and 100
    radar_return.amplitude = amplitude_dis(gen);

    // Add the return to the message
    radar_msg_.returns.push_back(radar_return);
  }

  // Publish the message
  radar_tracks_publisher_->publish(radar_msg_);

  // Log with colors (using ANSI escape codes)
  RCLCPP_INFO(this->get_logger(), 
              "\033[33mPublished random radar scan with %d returns\033[0m from: \033[31m%s\033[0m", 
              num_returns, radar_name_.c_str());
}

}  // namespace parameters_demo_nodes

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<parameters_demo_nodes::RadarScanDemoNode>();
    
    // Use MultiThreadedExecutor for improved performance
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    // Spin the executor
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error occurred: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  } catch (...) {
    std::cerr << "Unknown error occurred" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
}