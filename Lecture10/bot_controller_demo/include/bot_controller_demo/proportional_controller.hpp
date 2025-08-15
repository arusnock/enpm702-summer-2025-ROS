#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <memory>

namespace bot_controller_demo {

class ProportionalController : public rclcpp::Node
{
public:
    ProportionalController();
    
    void set_goal(double x, double y, double theta);
    void stop_robot();

private:
    void declare_parameter_with_description(const std::string& name, double default_value, const std::string& description);
    void declare_parameter_with_description(const std::string& name, const std::string& default_value, const std::string& description);
    void update_parameters();
    void publish_velocity(double linear_x, double angular_z);
    
    rcl_interfaces::msg::SetParametersResult parameter_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void control_callback();
    
    double normalize_angle(double angle) const;
    double get_distance_to_goal() const;
    double get_angle_to_goal() const;
    double get_angular_error() const;
    double get_final_angular_error() const;
    double clamp(double value, double min_val, double max_val) const;

    // Member variables - only create the publisher we need
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Parameters
    double kp_linear_;
    double kp_angular_;
    double goal_x_;
    double goal_y_;
    double goal_theta_;
    double linear_tolerance_;
    double angular_tolerance_;
    double control_frequency_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    std::string controller_type_;

    // State variables
    double current_x_;
    double current_y_;
    double current_theta_;
    bool pose_initialized_;
};

} // namespace bot_controller_demo
