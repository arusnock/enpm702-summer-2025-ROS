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
        default_value="rviz",
        description="Controller type: 'gazebo' or 'rviz'",
        choices=["gazebo", "rviz"]
    )
    
    linear_min_arg = DeclareLaunchArgument(
        "linear_min",
        default_value="0.1",
        description="Minimum linear velocity in m/s"
    )
    
    linear_max_arg = DeclareLaunchArgument(
        "linear_max",
        default_value="0.2",
        description="Maximum linear velocity in m/s"
    )
    
    angular_min_arg = DeclareLaunchArgument(
        "angular_min",
        default_value="-1.0",
        description="Minimum angular velocity in rad/s"
    )
    
    angular_max_arg = DeclareLaunchArgument(
        "angular_max",
        default_value="1.0",
        description="Maximum angular velocity in rad/s"
    )
    
    timer_period_ms_arg = DeclareLaunchArgument(
        "timer_period_ms",
        default_value="200",
        description="Timer period in milliseconds"
    )
    
    random_controller = Node(
        package="bot_controller_demo",
        executable="random_controller",
        name="random_controller",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'controller_type': LaunchConfiguration("controller_type"),
            'linear_min': LaunchConfiguration("linear_min"),
            'linear_max': LaunchConfiguration("linear_max"),
            'angular_min': LaunchConfiguration("angular_min"),
            'angular_max': LaunchConfiguration("angular_max"),
            'timer_period_ms': LaunchConfiguration("timer_period_ms")
        }]
    )
    
    # Add all launch arguments
    ld.add_action(controller_type_arg)
    ld.add_action(linear_min_arg)
    ld.add_action(linear_max_arg)
    ld.add_action(angular_min_arg)
    ld.add_action(angular_max_arg)
    ld.add_action(timer_period_ms_arg)
    
    # Add the node
    ld.add_action(random_controller)
    
    return ld