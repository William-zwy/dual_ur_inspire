from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mimic_joint_controller',
            executable='mimic_node',
            name='mimic_node',
            output='screen'
        )
    ])
