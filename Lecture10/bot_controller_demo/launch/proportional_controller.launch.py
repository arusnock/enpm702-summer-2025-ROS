from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# This function must be defined
def generate_launch_description():
    ld = LaunchDescription()
    
    # Declare launch arguments with default values
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="gazebo",
        description="Controller type: 'gazebo' or 'rviz'",
        choices=["gazebo", "rviz"]
    )
    
    kp_linear_arg = DeclareLaunchArgument(
        "kp_linear", 
        default_value="0.5", 
        description="Linear proportional gain"
    )
    
    kp_angular_arg = DeclareLaunchArgument(
        "kp_angular", 
        default_value="1.0", 
        description="Angular proportional gain"
    )
    
    goal_x_arg = DeclareLaunchArgument(
        "goal_x", 
        default_value="2.0", 
        description="Goal x-coordinate"
    )
    
    goal_y_arg = DeclareLaunchArgument(
        "goal_y", 
        default_value="1.0", 
        description="Goal y-coordinate"
    )
    
    goal_theta_arg = DeclareLaunchArgument(
        "goal_theta",
        default_value="1.5708",
        description="Goal orientation in radians (π/2 by default)"
    )
    
    linear_tolerance_arg = DeclareLaunchArgument(
        "linear_tolerance", 
        default_value="0.1", 
        description="Linear position tolerance"
    )
    
    angular_tolerance_arg = DeclareLaunchArgument(
        "angular_tolerance",
        default_value="0.05",
        description="Angular orientation tolerance"
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        "control_frequency",
        default_value="20.0",
        description="Control loop frequency in Hz"
    )
    
    max_linear_velocity_arg = DeclareLaunchArgument(
        "max_linear_velocity",
        default_value="0.5",
        description="Maximum linear velocity in m/s"
    )
    
    max_angular_velocity_arg = DeclareLaunchArgument(
        "max_angular_velocity",
        default_value="1.0",
        description="Maximum angular velocity in rad/s"
    )
    
    # Single universal controller node
    proportional_controller = Node(
        package="bot_controller_demo",
        executable="proportional_controller",
        name="proportional_controller",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "controller_type": LaunchConfiguration("controller_type"),
            "kp_linear": LaunchConfiguration("kp_linear"),
            "kp_angular": LaunchConfiguration("kp_angular"),
            "goal_x": LaunchConfiguration("goal_x"),
            "goal_y": LaunchConfiguration("goal_y"),
            "goal_theta": LaunchConfiguration("goal_theta"),
            "linear_tolerance": LaunchConfiguration("linear_tolerance"),
            "angular_tolerance": LaunchConfiguration("angular_tolerance"),
            "control_frequency": LaunchConfiguration("control_frequency"),
            "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
            "max_angular_velocity": LaunchConfiguration("max_angular_velocity")
        }]
    )
    
    # Add all launch arguments to launch description
    ld.add_action(controller_type_arg)
    ld.add_action(kp_linear_arg)
    ld.add_action(kp_angular_arg)
    ld.add_action(goal_x_arg)
    ld.add_action(goal_y_arg)
    ld.add_action(goal_theta_arg)
    ld.add_action(linear_tolerance_arg)
    ld.add_action(angular_tolerance_arg)
    ld.add_action(control_frequency_arg)
    ld.add_action(max_linear_velocity_arg)
    ld.add_action(max_angular_velocity_arg)
    
    # Add the single controller node
    ld.add_action(proportional_controller)
    
    return ld