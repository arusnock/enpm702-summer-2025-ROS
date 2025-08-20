#include "remapping_demo/camera.hpp"
#include "remapping_demo/color_utils.hpp"

#include <random>
#include <chrono>

namespace remapping_demo {

CameraDemoNode::CameraDemoNode() 
    : Node("camera_demo"), frame_counter_(0)
{
    // Declare parameters with descriptions and constraints
    rcl_interfaces::msg::ParameterDescriptor camera_name_desc;
    camera_name_desc.description = "Camera name";
    this->declare_parameter("camera_name", "camera", camera_name_desc);

    rcl_interfaces::msg::ParameterDescriptor camera_frame_desc;
    camera_frame_desc.description = "Name of the camera frame";
    this->declare_parameter("camera_frame", "camera_id", camera_frame_desc);

    rcl_interfaces::msg::ParameterDescriptor camera_rate_desc;
    camera_rate_desc.description = "Camera frame rate in Hz";
    rcl_interfaces::msg::IntegerRange rate_range;
    rate_range.from_value = 10;
    rate_range.to_value = 60;
    rate_range.step = 1;
    camera_rate_desc.integer_range.push_back(rate_range);
    this->declare_parameter("camera_rate", 60, camera_rate_desc);

    // Get parameters
    camera_name_ = this->get_parameter("camera_name").as_string();
    camera_frame_ = this->get_parameter("camera_frame").as_string();
    camera_rate_ = this->get_parameter("camera_rate").as_int();

    // Register parameter callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CameraDemoNode::parameter_update_callback, this, std::placeholders::_1));

    // Initialize image properties
    image_width_ = 160;   // 1/4 of the original width
    image_height_ = 120;  // 1/4 of the original height
    image_channels_ = 3;
    image_step_ = image_width_ * image_channels_;

    // Generate initial image data
    generate_image_data();

    // Set up publisher
    data_camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "camera/image_color", 10);

    // Set up timer
    data_camera_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / camera_rate_),
        std::bind(&CameraDemoNode::data_camera_pub_callback, this));

    RCLCPP_INFO(this->get_logger(), "Camera demo node initialized with name: %s, rate: %d Hz", 
                camera_name_.c_str(), camera_rate_);
}

void CameraDemoNode::generate_image_data()
{
    // Generate random image data
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint8_t> dis(0, 255);

    image_data_.resize(image_height_ * image_width_ * image_channels_);
    for (size_t i = 0; i < image_data_.size(); ++i) {
        image_data_[i] = dis(gen);
    }
}

rcl_interfaces::msg::SetParametersResult CameraDemoNode::parameter_update_callback(
    const std::vector<rclcpp::Parameter>& params)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "camera_name") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                camera_name_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Updated camera_name to: %s", camera_name_.c_str());
            } else {
                result.successful = false;
                result.reason = "camera_name must be a string";
                return result;
            }
        }
        else if (param.get_name() == "camera_rate") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
                int new_rate = param.as_int();
                if (new_rate >= 10 && new_rate <= 60) {
                    camera_rate_ = new_rate;
                    RCLCPP_INFO(this->get_logger(), "Updated camera_rate to: %d", camera_rate_);
                    
                    // Update timer with new rate
                    if (data_camera_timer_) {
                        data_camera_timer_->cancel();
                        data_camera_timer_ = this->create_wall_timer(
                            std::chrono::duration<double>(1.0 / camera_rate_),
                            std::bind(&CameraDemoNode::data_camera_pub_callback, this));
                        RCLCPP_INFO(this->get_logger(), "Timer rate updated to: %d Hz", camera_rate_);
                    }
                } else {
                    result.successful = false;
                    result.reason = "camera_rate must be between 10 and 60 Hz";
                    return result;
                }
            } else {
                result.successful = false;
                result.reason = "camera_rate must be an integer";
                return result;
            }
        }
        else if (param.get_name() == "camera_frame") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                camera_frame_ = param.as_string();
                RCLCPP_INFO(this->get_logger(), "Updated camera_frame to: %s", camera_frame_.c_str());
            } else {
                result.successful = false;
                result.reason = "camera_frame must be a string";
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
    data_camera_msg_.header.frame_id = camera_frame_;

    // Set image properties
    data_camera_msg_.height = image_height_;
    data_camera_msg_.width = image_width_;
    data_camera_msg_.encoding = "rgb8";
    data_camera_msg_.is_bigendian = false;
    data_camera_msg_.step = image_step_;

    // Set image data
    data_camera_msg_.data = image_data_;

    // Publish the message
    data_camera_publisher_->publish(data_camera_msg_);

    RCLCPP_INFO(this->get_logger(), "%sPublished random camera data from:%s %s%s%s",
                Color::PURPLE, Color::RESET, Color::RED, camera_name_.c_str(), Color::RESET);

    frame_counter_++;
}

} // namespace remapping_demo

int main(int argc, char** argv)
{
    try {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<remapping_demo::CameraDemoNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("camera_demo"), "Exception caught: %s", e.what());
    }

    RCLCPP_INFO(rclcpp::get_logger("camera_demo"), "Node stopped cleanly");
    rclcpp::shutdown();
    
    return 0;
}