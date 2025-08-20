#include "lifecycle_demo/sensor_publisher.hpp"
#include <rclcpp/rclcpp.hpp>

SensorPublisher::SensorPublisher() : LifecycleNode("sensor_publisher") {
  RCLCPP_INFO(get_logger(), "Sensor Publisher created");
}

// Configure transition: Unconfigured -> Inactive
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SensorPublisher::on_configure(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(get_logger(), "Configuring from: %s",
              previous_state.label().c_str());

  // Create publisher (but don't activate yet)
  publisher_ = create_publisher<std_msgs::msg::String>("sensor_data", 10);

  RCLCPP_INFO(get_logger(), "Configuration complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

// Activate transition: Inactive -> Active
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SensorPublisher::on_activate(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(get_logger(), "Activating from: %s",
              previous_state.label().c_str());

  // Activate publisher and start timer
  publisher_->on_activate();

  timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
    auto msg = std_msgs::msg::String();
    msg.data = "Sensor reading: " + std::to_string(counter_++);
    publisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published: %s", msg.data.c_str());
  });

  RCLCPP_INFO(get_logger(), "Activation complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

// Deactivate transition: Active -> Inactive
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SensorPublisher::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(get_logger(), "Deactivating from: %s",
              previous_state.label().c_str());

  // Stop timer and deactivate publisher
  timer_.reset();
  publisher_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivation complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

// Cleanup transition: Inactive -> Unconfigured
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SensorPublisher::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(get_logger(), "Cleaning up from: %s",
              previous_state.label().c_str());

  // Reset all resources
  timer_.reset();
  publisher_.reset();
  counter_ = 0;

  RCLCPP_INFO(get_logger(), "Cleanup complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

// Shutdown transition: Any state -> Finalized
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SensorPublisher::on_shutdown(const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(get_logger(), "Shutting down from: %s",
              previous_state.label().c_str());

  // Emergency cleanup
  if (timer_)
    timer_.reset();
  if (publisher_)
    publisher_.reset();
  counter_ = 0;

  RCLCPP_INFO(get_logger(), "Shutdown complete");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorPublisher>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}