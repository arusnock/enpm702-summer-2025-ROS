from launch import LaunchDescription
from launch_ros.actions import Node

# This function must be defined
def generate_launch_description():
    ld = LaunchDescription()
    
    random_controller = Node(
        package="bot_controller_demo",
        executable="random_controller",
        name="random_controller",
        output="screen"
        # Uncomment and modify if remappings are needed:
        # remappings=[
        #     ('/cmd_vel', '/robot1/cmd_vel'),
        #     ('/odom', '/robot1/odom'),
        # ]
    )
    
    ld.add_action(random_controller)
    return ld