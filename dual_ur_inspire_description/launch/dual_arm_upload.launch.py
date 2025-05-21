from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('dual_ur_inspire_description')
    # xacro_pkg = get_package_share_directory('xacro') # Not needed if xacro is in PATH

    limited_arg = DeclareLaunchArgument(
        'limited',
        default_value='false',
        description='If true, limits joint range [-PI, PI] on all joints.'
    )

    transmission_hw_interface_arg = DeclareLaunchArgument(
        'transmission_hw_interface',
        default_value='hardware_interface/PositionJointInterface',
        description='Hardware interface for transmission.'
    )

    robot_description_content = ParameterValue(
        Command([
            'xacro', # Use xacro directly from PATH
            ' ',
            os.path.join(pkg_dir, 'urdf', 'dual_ur_robotiq.urdf.xacro'),
            ' transmission_hw_interface:=', LaunchConfiguration('transmission_hw_interface'),
            ' limited:=', LaunchConfiguration('limited') # Assuming 'limited' is a parameter for your xacro
        ]),
        value_type=str
    )

    return LaunchDescription([
        limited_arg,
        transmission_hw_interface_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ])
