#include "parameters_demo/camera_demo.hpp"
#include <random>
#include <chrono>

namespace parameters_demo_nodes
{

CameraDemoNode::CameraDemoNode()
: Node("camera_demo"), 
  camera_name_("camera"),
  camera_rate_(60),
  frame_counter_(0)
{

  // Declare all parameters for the node
  
  // Declare camera_name parameter
  rcl_interfaces::msg::ParameterDescriptor camera_name_desc;
  camera_name_desc.description = "Camera name";
  this->declare_parameter("camera_name", "cameraX", camera_name_desc);

  // Declare camera_rate parameter with integer range constraint
  rcl_interfaces::msg::ParameterDescriptor camera_rate_desc;
  camera_rate_desc.description = "Camera frame rate in Hz";
  rcl_interfaces::msg::IntegerRange rate_range;
  rate_range.from_value = 10;
  rate_range.to_value = 60;
  rate_range.step = 1;
  camera_rate_desc.integer_range.push_back(rate_range);
  this->declare_parameter("camera_rate", 60, camera_rate_desc);

  // Get Parameters
  camera_name_ = this->get_parameter("camera_name").get_parameter_value().get<std::string>();
  camera_rate_ = this->get_parameter("camera_rate").as_int();

  // Register parameter callback for dynamic parameter updates
  // param_callback_handle_ = this->add_on_set_parameters_callback(
  //   std::bind(&CameraDemoNode::parameter_update_callback, this, std::placeholders::_1));

  // Generate initial random image data
  generate_random_image();

  // Set up publisher for camera images
  data_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_color", 10);

  // Set up timer for periodic publishing
  auto timer_period = std::chrono::duration<double>(1.0 / camera_rate_);
  data_camera_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&CameraDemoNode::data_camera_pub_callback, this));

  RCLCPP_INFO(this->get_logger(), "%sCamera demo node initialized with name: %s, rate: %d Hz%s", 
              Color::CYAN, camera_name_.c_str(), camera_rate_, Color::RESET);
}

rcl_interfaces::msg::SetParametersResult CameraDemoNode::parameter_update_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : params) {
    if (param.get_name() == "camera_name") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        camera_name_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "%sUpdated camera_name to: %s%s", 
                    Color::GREEN, camera_name_.c_str(), Color::RESET);
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for camera_name";
        return result;
      }
    } else if (param.get_name() == "camera_rate") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        int new_rate = param.as_int();
        if (new_rate >= 10 && new_rate <= 60) {
          camera_rate_ = new_rate;
          RCLCPP_INFO(this->get_logger(), "%sUpdated camera_rate to: %d%s", 
                      Color::GREEN, camera_rate_, Color::RESET);
          
          // Update timer with new rate
          if (data_camera_timer_) {
            data_camera_timer_.reset();
            auto timer_period = std::chrono::duration<double>(1.0 / camera_rate_);
            data_camera_timer_ = this->create_wall_timer(
              std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
              std::bind(&CameraDemoNode::data_camera_pub_callback, this));
            RCLCPP_INFO(this->get_logger(), "%sTimer rate updated to: %d Hz%s", 
                        Color::BLUE, camera_rate_, Color::RESET);
          }
        } else {
          result.successful = false;
          result.reason = "camera_rate must be between 10 and 60 Hz";
          return result;
        }
      } else {
        result.successful = false;
        result.reason = "Invalid parameter type for camera_rate";
        return result;
      }
    }
  }

  return result;
}

void CameraDemoNode::data_camera_pub_callback()
{
  // Set header information
  data_camera_msg_.header.stamp = this->get_clock()->now();
  data_camera_msg_.header.frame_id = "camera_frame";

  // Set image metadata
  data_camera_msg_.height = image_height_;
  data_camera_msg_.width = image_width_;
  data_camera_msg_.encoding = "rgb8";
  data_camera_msg_.is_bigendian = false;
  data_camera_msg_.step = image_step_;

  // Set image data
  data_camera_msg_.data = image_data_;

  // Publish the message
  data_camera_publisher_->publish(data_camera_msg_);

  // Log with colors (using ANSI escape codes)
  RCLCPP_INFO(this->get_logger(), 
              "%sPublished frame from:%s %s%s%s", 
              parameters_demo_nodes::Color::PURPLE, parameters_demo_nodes::Color::RESET,
              parameters_demo_nodes::Color::RED, camera_name_.c_str(), parameters_demo_nodes::Color::RESET);

  frame_counter_++;
}

void CameraDemoNode::generate_random_image()
{
  // Initialize random number generator
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<uint8_t> dis(0, 255);

  // Resize vector to hold image data
  image_data_.resize(image_height_ * image_width_ * image_channels_);

  // Fill with random RGB values
  for (size_t i = 0; i < image_data_.size(); ++i) {
    image_data_[i] = dis(gen);
  }
}

}  // namespace parameters_demo_nodes

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<parameters_demo_nodes::CameraDemoNode>();
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