#include "parameters_demo/processing_demo.hpp"
#include <chrono>

namespace parameters_demo_nodes
{

ProcessingDemoNode::ProcessingDemoNode()
: Node("processing_demo"),
  processing_mode_("all"),
  processing_rate_(10),
  camera_frames_received_(0),
  lidar_scans_received_(0),
  radar_scans_received_(0),
  processing_cycles_(0)
{
  // Define callback groups for different sensor types
  sensor_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  processing_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Declare all parameters for the node
  rcl_interfaces::msg::ParameterDescriptor processing_mode_desc;
  processing_mode_desc.description = "Processing mode (all, radar_only, camera_only, lidar_only)";
  this->declare_parameter("processing_mode", "all", processing_mode_desc);

  rcl_interfaces::msg::ParameterDescriptor processing_rate_desc;
  processing_rate_desc.description = "Processing rate in Hz";
  rcl_interfaces::msg::IntegerRange rate_range;
  rate_range.from_value = 1;
  rate_range.to_value = 30;
  rate_range.step = 1;
  processing_rate_desc.integer_range.push_back(rate_range);
  this->declare_parameter("processing_rate", 10, processing_rate_desc);

  // Get Parameters
  processing_mode_ = this->get_parameter("processing_mode").as_string();
  processing_rate_ = this->get_parameter("processing_rate").as_int();

  // Create QoS profile for better reliability in sensor data
  rclcpp::QoS sensor_qos(10);
  sensor_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  sensor_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  // Create subscribers for each sensor
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sensor_cb_group_;

  camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_color",
    sensor_qos,
    std::bind(&ProcessingDemoNode::camera_callback, this, std::placeholders::_1),
    sub_options);

  lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar/points",
    sensor_qos,
    std::bind(&ProcessingDemoNode::lidar_callback, this, std::placeholders::_1),
    sub_options);

  radar_subscriber_ = this->create_subscription<radar_msgs::msg::RadarScan>(
    "/radar/tracks",
    sensor_qos,
    std::bind(&ProcessingDemoNode::radar_callback, this, std::placeholders::_1),
    sub_options);

  // Set up processing timer
  auto processing_period = std::chrono::duration<double>(1.0 / processing_rate_);
  processing_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(processing_period),
    std::bind(&ProcessingDemoNode::process_data_callback, this),
    processing_cb_group_);

  // Log with colors (using ANSI escape codes)
  RCLCPP_INFO(this->get_logger(), 
              "%sProcessing Demo Node initialized with mode: %s%s", 
              Color::CYAN, processing_mode_.c_str(), Color::RESET);
  RCLCPP_INFO(this->get_logger(), 
              "%sProcessing rate: %d Hz%s", 
              Color::CYAN, processing_rate_, Color::RESET);
}

void ProcessingDemoNode::camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  latest_camera_data_ = msg;
  camera_frames_received_++;

  // Log at reduced frequency to avoid console spam
  if (camera_frames_received_ % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(),
                 "%sReceived camera frame #%lu size: %ux%u%s",
                 Color::PURPLE, camera_frames_received_, 
                 msg->width, msg->height, Color::RESET);
  }
}

void ProcessingDemoNode::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_lidar_data_ = msg;
  lidar_scans_received_++;

  // Log at reduced frequency to avoid console spam
  if (lidar_scans_received_ % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(),
                 "%sReceived lidar scan #%lu with %lu points%s",
                 Color::GREEN, lidar_scans_received_, 
                 msg->ranges.size(), Color::RESET);
  }
}

void ProcessingDemoNode::radar_callback(const radar_msgs::msg::RadarScan::SharedPtr msg)
{
  latest_radar_data_ = msg;
  radar_scans_received_++;

  // Log at reduced frequency to avoid console spam
  if (radar_scans_received_ % 10 == 0) {
    RCLCPP_DEBUG(this->get_logger(),
                 "%sReceived radar scan #%lu with %lu returns%s",
                 Color::CYAN, radar_scans_received_, 
                 msg->returns.size(), Color::RESET);
  }
}

void ProcessingDemoNode::process_data_callback()
{
  processing_cycles_++;

  // Skip processing if data is not available based on mode
  if (processing_mode_ == "all") {
    if (!latest_camera_data_ || !latest_lidar_data_ || !latest_radar_data_) {
      return;
    }
  } else if (processing_mode_ == "camera_only" && !latest_camera_data_) {
    return;
  } else if (processing_mode_ == "lidar_only" && !latest_lidar_data_) {
    return;
  } else if (processing_mode_ == "radar_only" && !latest_radar_data_) {
    return;
  }

  // Perform data processing based on mode
  // This is where you would implement your sensor data processing algorithm

  // Print status every few cycles
  if (processing_cycles_ % 5 == 0) {
    print_processing_status();
  }
}

void ProcessingDemoNode::print_processing_status()
{
  std::string status_header = std::string("\n") + Color::YELLOW + 
                             "===== Processing Status =====" + Color::RESET + "\n";
  std::string status_footer = std::string(Color::YELLOW) + 
                             "=============================\n" + Color::RESET;

  // Common information for all modes
  std::string status_info = std::string(Color::RED) + "Processing cycle: " + 
                           std::to_string(processing_cycles_) + Color::RESET + "\n";

  // Mode-specific information
  if (processing_mode_ == "all") {
    status_info += std::string(Color::PURPLE) + "Camera frames received: " + 
                   std::to_string(camera_frames_received_) + Color::RESET + "\n";
    status_info += std::string(Color::GREEN) + "Lidar scans received: " + 
                   std::to_string(lidar_scans_received_) + Color::RESET + "\n";
    status_info += std::string(Color::CYAN) + "Radar scans received: " + 
                   std::to_string(radar_scans_received_) + Color::RESET + "\n";
  } else if (processing_mode_ == "camera_only") {
    status_info += std::string(Color::PURPLE) + "Camera frames received: " + 
                   std::to_string(camera_frames_received_) + Color::RESET + "\n";
  } else if (processing_mode_ == "lidar_only") {
    status_info += std::string(Color::GREEN) + "Lidar scans received: " + 
                   std::to_string(lidar_scans_received_) + Color::RESET + "\n";
  } else if (processing_mode_ == "radar_only") {
    status_info += std::string(Color::CYAN) + "Radar scans received: " + 
                   std::to_string(radar_scans_received_) + Color::RESET + "\n";
  }

  // Print complete status with header and footer
  RCLCPP_INFO(this->get_logger(), "%s", (status_header + status_info + status_footer).c_str());
}

}  // namespace parameters_demo_nodes

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    
    // Create the node
    auto node = std::make_shared<parameters_demo_nodes::ProcessingDemoNode>();
    
    // Create a MultiThreadedExecutor with specified number of threads
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
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