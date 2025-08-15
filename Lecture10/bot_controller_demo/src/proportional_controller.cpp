#include "bot_controller_demo/proportional_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

namespace bot_controller_demo {

ProportionalController::ProportionalController() 
    : Node("proportional_controller"), pose_initialized_(false)
{
    // Declare parameters with descriptions
    declare_parameter_with_description("kp_linear", 0.5, "Linear proportional gain");
    declare_parameter_with_description("kp_angular", 1.0, "Angular proportional gain");
    declare_parameter_with_description("goal_x", 2.0, "Goal x-coordinate");
    declare_parameter_with_description("goal_y", 1.0, "Goal y-coordinate");
    declare_parameter_with_description("goal_theta", M_PI / 2.0, "Goal orientation in radians");
    declare_parameter_with_description("linear_tolerance", 0.1, "Linear position tolerance");
    declare_parameter_with_description("angular_tolerance", 0.05, "Angular orientation tolerance");
    declare_parameter_with_description("control_frequency", 20.0, "Control loop frequency");
    declare_parameter_with_description("max_linear_velocity", 0.5, "Maximum linear velocity");
    declare_parameter_with_description("max_angular_velocity", 1.0, "Maximum angular velocity");
    declare_parameter_with_description("controller_type", "rviz", "Controller type: 'rviz' for Twist messages, 'gazebo' for TwistStamped messages");

    // Get parameters
    update_parameters();

    // Initialize pose variables
    current_x_ = 0.0;
    current_y_ = 0.0;
    current_theta_ = 0.0;

    // Create the appropriate publisher based on controller_type parameter
    if (controller_type_ == "gazebo") {
        cmd_vel_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Created TwistStamped publisher for Gazebo");
    } else if (controller_type_ == "rviz") {
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Created Twist publisher for RViz");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid controller_type '%s'. Valid options are 'rviz' or 'gazebo'. Defaulting to 'rviz'.", controller_type_.c_str());
        controller_type_ = "rviz";
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Created Twist publisher for RViz (default)");
    }
    
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ProportionalController::odom_callback, this, std::placeholders::_1));

    // Control timer
    auto timer_period = std::chrono::duration<double>(1.0 / control_frequency_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&ProportionalController::control_callback, this));

    // Parameter callback for dynamic reconfiguration
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ProportionalController::parameter_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
               "Proportional Controller initialized with goal: x=%.2f, y=%.2f, theta=%.2f",
               goal_x_, goal_y_, goal_theta_);
    RCLCPP_INFO(this->get_logger(),
               "Linear gain: Kp=%.2f, Angular gain: Kp=%.2f",
               kp_linear_, kp_angular_);
    RCLCPP_INFO(this->get_logger(),
               "Publishing %s messages", (controller_type_ == "gazebo") ? "TwistStamped (Gazebo)" : "Twist (RViz)");
}

void ProportionalController::set_goal(double x, double y, double theta)
{
    goal_x_ = x;
    goal_y_ = y;
    goal_theta_ = theta;
    RCLCPP_INFO(this->get_logger(),
               "Goal updated to: x=%.2f, y=%.2f, theta=%.2f",
               goal_x_, goal_y_, goal_theta_);
}

void ProportionalController::stop_robot()
{
    publish_velocity(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Robot stopped");
}

void ProportionalController::declare_parameter_with_description(const std::string& name, double default_value, const std::string& description)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    this->declare_parameter(name, default_value, desc);
}

void ProportionalController::declare_parameter_with_description(const std::string& name, const std::string& default_value, const std::string& description)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;
    this->declare_parameter(name, default_value, desc);
}

void ProportionalController::update_parameters()
{
    kp_linear_ = this->get_parameter("kp_linear").as_double();
    kp_angular_ = this->get_parameter("kp_angular").as_double();
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    goal_theta_ = this->get_parameter("goal_theta").as_double();
    linear_tolerance_ = this->get_parameter("linear_tolerance").as_double();
    angular_tolerance_ = this->get_parameter("angular_tolerance").as_double();
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    controller_type_ = this->get_parameter("controller_type").as_string();
}

void ProportionalController::publish_velocity(double linear_x, double angular_z)
{
    if (controller_type_ == "gazebo") {
        // Publish TwistStamped for Gazebo
        if (cmd_vel_stamped_publisher_) {
            auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
            twist_stamped_msg.header.stamp = this->get_clock()->now();
            twist_stamped_msg.header.frame_id = "base_link";
            twist_stamped_msg.twist.linear.x = linear_x;
            twist_stamped_msg.twist.angular.z = angular_z;
            cmd_vel_stamped_publisher_->publish(twist_stamped_msg);
        }
    } else {
        // Publish regular Twist for RViz (default)
        if (cmd_vel_publisher_) {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = linear_x;
            twist_msg.angular.z = angular_z;
            cmd_vel_publisher_->publish(twist_msg);
        }
    }
}

rcl_interfaces::msg::SetParametersResult ProportionalController::parameter_callback(
    const std::vector<rclcpp::Parameter>& parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "kp_linear" || param.get_name() == "kp_angular" ||
            param.get_name() == "goal_x" || param.get_name() == "goal_y" ||
            param.get_name() == "goal_theta" || param.get_name() == "linear_tolerance" ||
            param.get_name() == "angular_tolerance" || param.get_name() == "control_frequency" ||
            param.get_name() == "max_linear_velocity" || param.get_name() == "max_angular_velocity") {
            
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason = "Parameter " + param.get_name() + " must be of type double";
                return result;
            }
        } else if (param.get_name() == "controller_type") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                result.successful = false;
                result.reason = "Parameter " + param.get_name() + " must be of type string";
                return result;
            }
            
            std::string new_type = param.as_string();
            if (new_type != "rviz" && new_type != "gazebo") {
                result.successful = false;
                result.reason = "Parameter controller_type must be either 'rviz' or 'gazebo'";
                return result;
            }
            
            RCLCPP_WARN(this->get_logger(), 
                       "Changing controller_type parameter at runtime is not supported. "
                       "Restart the node to apply changes.");
        }
    }

    if (result.successful) {
        update_parameters();
        RCLCPP_INFO(this->get_logger(), "Parameters updated successfully");
    }

    return result;
}

void ProportionalController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    // Convert quaternion to Euler angles
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_theta_);

    if (!pose_initialized_) {
        pose_initialized_ = true;
        RCLCPP_INFO(this->get_logger(),
                   "Initial pose: x=%.2f, y=%.2f, theta=%.2f",
                   current_x_, current_y_, current_theta_);
    }
}

double ProportionalController::normalize_angle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

double ProportionalController::get_distance_to_goal() const
{
    return std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
}

double ProportionalController::get_angle_to_goal() const
{
    return std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
}

double ProportionalController::get_angular_error() const
{
    double angle_to_goal = get_angle_to_goal();
    return normalize_angle(angle_to_goal - current_theta_);
}

double ProportionalController::get_final_angular_error() const
{
    return normalize_angle(goal_theta_ - current_theta_);
}

double ProportionalController::clamp(double value, double min_val, double max_val) const
{
    return std::max(min_val, std::min(value, max_val));
}

void ProportionalController::control_callback()
{
    if (!pose_initialized_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Waiting for odometry data...");
        return;
    }

    double distance_error = get_distance_to_goal();
    double angular_error = get_angular_error();
    double final_angular_error = get_final_angular_error();

    // Check if goal is reached (both position and orientation)
    bool position_reached = distance_error <= linear_tolerance_;
    bool orientation_reached = std::abs(final_angular_error) <= angular_tolerance_;

    if (position_reached && orientation_reached) {
        // Goal completely reached - stop robot and shutdown
        publish_velocity(0.0, 0.0);
        
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully! Shutting down node...");
        RCLCPP_INFO(this->get_logger(), 
                   "Final pose: x=%.3f, y=%.3f, theta=%.3f", 
                   current_x_, current_y_, current_theta_);
        RCLCPP_INFO(this->get_logger(), 
                   "Final errors: distance=%.3f, angle=%.3f", 
                   distance_error, final_angular_error);
        
        // Cancel the timer and initiate shutdown
        timer_->cancel();
        rclcpp::shutdown();
        return;
    }

    double linear_vel = 0.0;
    double angular_vel = 0.0;

    // Combined movement - always try to move and turn simultaneously
    if (distance_error > linear_tolerance_) {
        // Calculate linear velocity based on distance error
        linear_vel = kp_linear_ * distance_error;

        // While moving, blend the angular control between:
        // - Heading to goal position when far
        // - Gradually adjusting to final orientation as we get closer
        double position_weight = std::min(1.0, distance_error / 0.5);  // Full weight until 0.5m
        double orientation_weight = 1.0 - position_weight;

        // Blend the two angular errors
        double blended_error = (position_weight * angular_error) + 
                             (orientation_weight * final_angular_error);
        angular_vel = kp_angular_ * blended_error;

        RCLCPP_DEBUG(this->get_logger(),
                    "Moving: distance=%.2f, blended_angle=%.2f, weights: pos=%.2f, orient=%.2f",
                    distance_error, blended_error, position_weight, orientation_weight);
    } else {
        // At goal position, only adjust orientation
        linear_vel = 0.0;
        angular_vel = kp_angular_ * final_angular_error;
        
        RCLCPP_DEBUG(this->get_logger(),
                    "At goal, adjusting orientation: error=%.2f", final_angular_error);
    }

    // Apply velocity limits
    linear_vel = clamp(linear_vel, -max_linear_velocity_, max_linear_velocity_);
    angular_vel = clamp(angular_vel, -max_angular_velocity_, max_angular_velocity_);

    // Publish velocity command
    publish_velocity(linear_vel, angular_vel);

    // Log progress periodically (about once per second)
    static auto last_log_time = this->get_clock()->now();
    auto current_time = this->get_clock()->now();
    if ((current_time - last_log_time).seconds() >= 1.0) {
        RCLCPP_INFO(this->get_logger(),
                   "Current: x=%.2f, y=%.2f, theta=%.2f, distance=%.2f, angle_error=%.2f",
                   current_x_, current_y_, current_theta_, distance_error, angular_error);
        last_log_time = current_time;
    }
}

} // namespace bot_controller_demo

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto controller = std::make_shared<bot_controller_demo::ProportionalController>();

    try {
        rclcpp::spin(controller);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(controller->get_logger(), "Exception caught: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(controller->get_logger(), "Unknown exception caught");
    }

    // Stop robot before shutdown
    controller->stop_robot();
    rclcpp::shutdown();
}