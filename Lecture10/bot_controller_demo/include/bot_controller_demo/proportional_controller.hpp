#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <cmath>
#include <memory>

class ProportionalController : public rclcpp::Node
{
public:
    ProportionalController() : Node("proportional_controller"), pose_initialized_(false)
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
        declare_parameter_with_description("publish_stamped", false, "Publish TwistStamped for Gazebo compatibility");

        // Get parameters
        update_parameters();

        // Initialize pose variables
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_theta_ = 0.0;

        // Create the appropriate publisher based on publish_stamped parameter
        if (publish_stamped_) {
            cmd_vel_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "Created TwistStamped publisher for Gazebo");
        } else {
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "Created Twist publisher for RViz");
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
                   "Publishing %s messages", publish_stamped_ ? "TwistStamped (Gazebo)" : "Twist (RViz)");
    }

    void set_goal(double x, double y, double theta)
    {
        goal_x_ = x;
        goal_y_ = y;
        goal_theta_ = theta;
        RCLCPP_INFO(this->get_logger(),
                   "Goal updated to: x=%.2f, y=%.2f, theta=%.2f",
                   goal_x_, goal_y_, goal_theta_);
    }

    void stop_robot()
    {
        publish_velocity(0.0, 0.0);
        RCLCPP_INFO(this->get_logger(), "Robot stopped");
    }

private:
    void declare_parameter_with_description(const std::string& name, double default_value, const std::string& description)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = description;
        this->declare_parameter(name, default_value, desc);
    }

    void declare_parameter_with_description(const std::string& name, bool default_value, const std::string& description)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;
        desc.description = description;
        this->declare_parameter(name, default_value, desc);
    }

    void update_parameters()
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
        publish_stamped_ = this->get_parameter("publish_stamped").as_bool();
    }

    void publish_velocity(double linear_x, double angular_z)
    {
        if (publish_stamped_) {
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
            // Publish regular Twist for RViz
            if (cmd_vel_publisher_) {
                auto twist_msg = geometry_msgs::msg::Twist();
                twist_msg.linear.x = linear_x;
                twist_msg.angular.z = angular_z;
                cmd_vel_publisher_->publish(twist_msg);
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult parameter_callback(
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
            } else if (param.get_name() == "publish_stamped") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = "Parameter " + param.get_name() + " must be of type bool";
                    return result;
                }
                // Note: Changing publish_stamped at runtime would require recreating publishers
                // which is not supported in this implementation
                RCLCPP_WARN(this->get_logger(), 
                           "Changing publish_stamped parameter at runtime is not supported. "
                           "Restart the node to apply changes.");
            }
        }

        if (result.successful) {
            update_parameters();
            RCLCPP_INFO(this->get_logger(), "Parameters updated successfully");
        }

        return result;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

    double normalize_angle(double angle) const
    {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    double get_distance_to_goal() const
    {
        return std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    }

    double get_angle_to_goal() const
    {
        return std::atan2(goal_y_ - current_y_, goal_x_ - current_x_);
    }

    double get_angular_error() const
    {
        double angle_to_goal = get_angle_to_goal();
        return normalize_angle(angle_to_goal - current_theta_);
    }

    double get_final_angular_error() const
    {
        return normalize_angle(goal_theta_ - current_theta_);
    }

    double clamp(double value, double min_val, double max_val) const
    {
        return std::max(min_val, std::min(value, max_val));
    }

    void control_callback()
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
    bool publish_stamped_;

    // State variables
    double current_x_;
    double current_y_;
    double current_theta_;
    bool pose_initialized_;
};