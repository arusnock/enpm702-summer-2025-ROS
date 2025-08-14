#include "parameters_demo/lidar_demo.hpp"
#include <random>
#include <chrono>
#include <cmath>

namespace parameters_demo_nodes
{

LidarDemoNode::LidarDemoNode()
: Node("lidar_demo"), 
  lidar_name_("lidar"),
  lidar_model_("OS2"),
  lidar_rate_(20)
{
  // Declare all parameters for the node
  
  // Declare lidar_name parameter
  rcl_interfaces::msg::ParameterDescriptor lidar_name_desc;
  lidar_name_desc.description = "Lidar name";
  this->declare_parameter("lidar_name", "lidar", lidar_name_desc);

  // Declare lidar_model parameter
  rcl_interfaces::msg::ParameterDescriptor lidar_model_desc;
  lidar_model_desc.description = "Lidar model";
  this->declare_parameter("lidar_model", "OS2", lidar_model_desc);

  // Declare lidar_rate parameter with integer range constraint
  rcl_interfaces::msg::ParameterDescriptor lidar_rate_desc;
  lidar_rate_desc.description = "Lidar frame rate in Hz";
  rcl_interfaces::msg::IntegerRange rate_range;
  rate_range.from_value = 20;
  rate_range.to_value = 100;
  rate_range.step = 20;
  lidar_rate_desc.integer_range.push_back(rate_range);
  this->declare_parameter("lidar_rate", 20, lidar_rate_desc);

  // Get Parameters
  lidar_name_ = this->get_parameter("lidar_name").as_string();
  lidar_model_ = this->get_parameter("lidar_model").as_string();
  lidar_rate_ = this->get_parameter("lidar_rate").as_int();

  // Register parameter callback for dynamic parameter updates
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LidarDemoNode::parameter_update_callback, this, std::placeholders::_1));

  // Set up publisher for lidar scans
  data_lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
    "/lidar/points", 10);

  // Set up timer for periodic publishing
  auto timer_period = std::chrono::duration<double>(1.0 / lidar_rate_);
  data_lidar_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&LidarDemoNode::data_lidar_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "Lidar demo node initialized with name: %s, model: %s, rate: %d Hz", 
              lidar_name_.c_str(), lidar_model_.c_str(), lidar_rate_);
}

rcl_interfaces::msg::SetParametersResult LidarDemoNode::parameter_update_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : params) {
    if (param.get_name() == "lidar_name") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        lidar_name_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Updated lidar_name to: %s", lidar_name_.c_str());
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for lidar_name";
        return result;
      }
    } else if (param.get_name() == "lidar_model") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        lidar_model_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Updated lidar_model to: %s", lidar_model_.c_str());
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for lidar_model";
        return result;
      }
    } else if (param.get_name() == "lidar_rate") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int new_rate = param.as_int();
        if (new_rate >= 20 && new_rate <= 100 && new_rate % 20 == 0) {
          lidar_rate_ = new_rate;
          RCLCPP_INFO(this->get_logger(), "Updated lidar_rate to: %d", lidar_rate_);
          
          // Update timer with new rate
          if (data_lidar_timer_) {
            data_lidar_timer_.reset();
            auto timer_period = std::chrono::duration<double>(1.0 / lidar_rate_);
            data_lidar_timer_ = this->create_wall_timer(
              std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
              std::bind(&LidarDemoNode::data_lidar_pub_callback, this));
            RCLCPP_INFO(this->get_logger(), "Timer rate updated to: %d Hz", lidar_rate_);
          }
        } else {
          result.successful = false;
          result.reason = "lidar_rate must be between 20 and 100 Hz in 20 Hz increments";
          return result;
        }
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for lidar_rate";
        return result;
      }
    }
  }

  return result;
}

void LidarDemoNode::data_lidar_pub_callback()
{
  // Set header information
  data_lidar_msg_.header.stamp = this->get_clock()->now();
  data_lidar_msg_.header.frame_id = "laser_frame";

  // Set the angle and range parameters
  data_lidar_msg_.angle_min = -M_PI;
  data_lidar_msg_.angle_max = M_PI;
  data_lidar_msg_.angle_increment = M_PI / 180.0;  // 1 degree increments
  data_lidar_msg_.time_increment = 0.0;
  data_lidar_msg_.scan_time = 0.1;
  data_lidar_msg_.range_min = 0.2;
  data_lidar_msg_.range_max = 10.0;

  // Number of readings based on angle range
  int num_readings = static_cast<int>((data_lidar_msg_.angle_max - data_lidar_msg_.angle_min) / 
                                      data_lidar_msg_.angle_increment);

  // Initialize random number generators
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_real_distribution<float> range_dis(data_lidar_msg_.range_min, data_lidar_msg_.range_max);
  static std::uniform_real_distribution<float> intensity_dis(0.0f, 1.0f);

  // Generate random ranges and intensities
  data_lidar_msg_.ranges.clear();
  data_lidar_msg_.intensities.clear();
  data_lidar_msg_.ranges.reserve(num_readings);
  data_lidar_msg_.intensities.reserve(num_readings);

  for (int i = 0; i < num_readings; ++i) {
    data_lidar_msg_.ranges.push_back(range_dis(gen));
    data_lidar_msg_.intensities.push_back(intensity_dis(gen));
  }

  // Publish the message
  data_lidar_publisher_->publish(data_lidar_msg_);

  // Log with colors (using ANSI escape codes)
  RCLCPP_INFO(this->get_logger(), 
              "\033[32mPublished random lidar (%s) scan from: \033[0m\033[31m%s\033[0m", 
              lidar_model_.c_str(), lidar_name_.c_str());
}

}  // namespace parameters_demo_nodes

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<parameters_demo_nodes::LidarDemoNode>();
    rclcpp::spin(node);
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