#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <random>
#include <chrono>

class RandomController : public rclcpp::Node
{
public:
    RandomController() : Node("random_controller"), gen_(rd_())
    {
        // Create publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&RandomController::odom_callback, this, std::placeholders::_1));
        
        // Create timer to publish random velocities
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),  // Publish every 200ms
            std::bind(&RandomController::timer_callback, this));
        
        // Initialize random distributions
        linear_dist_ = std::uniform_real_distribution<double>(0.1, 0.2);   // 0.1 to 0.2 m/s
        angular_dist_ = std::uniform_real_distribution<double>(-1.0, 1.0);  // -1.0 to 1.0 rad/s
        
        RCLCPP_INFO(this->get_logger(), "Random Controller Node Started");
        RCLCPP_INFO(this->get_logger(), "Publishing random velocities to /cmd_vel");
        RCLCPP_INFO(this->get_logger(), "Monitoring robot pose from /odom");
    }

private:
    void timer_callback()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        
        // Generate random linear and angular velocities
        twist_msg.linear.x = linear_dist_(gen_);
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;
        
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_dist_(gen_);
        
        // Publish the velocity command
        cmd_vel_pub_->publish(twist_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
                    "Published velocity - Linear: %.3f, Angular: %.3f", 
                    twist_msg.linear.x, twist_msg.angular.z);
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract position
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        // Extract orientation and convert quaternion to Euler angles
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Display robot pose information
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,  // Throttle to 1Hz
                            "Robot Pose - Position: [%.3f, %.3f, %.3f], "
                            "Orientation: [R=%.3f, P=%.3f, Y=%.3f] rad",
                            x, y, z, roll, pitch, yaw);
        
        // Also display in degrees for easier understanding
        RCLCPP_DEBUG(this->get_logger(),
                    "Orientation in degrees: [R=%.1f°, P=%.1f°, Y=%.1f°]",
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
    }
    
    // Member variables
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Random number generation
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> linear_dist_;
    std::uniform_real_distribution<double> angular_dist_;
};