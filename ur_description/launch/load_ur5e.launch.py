from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ur_description')
    
    # Define launch arguments
    joint_limit_params = LaunchConfiguration('joint_limit_params', 
        default=os.path.join(pkg_dir, 'config/ur5e/joint_limits.yaml'))
    kinematics_params = LaunchConfiguration('kinematics_params',
        default=os.path.join(pkg_dir, 'config/ur5e/default_kinematics.yaml'))
    physical_params = LaunchConfiguration('physical_params',
        default=os.path.join(pkg_dir, 'config/ur5e/physical_parameters.yaml'))
    visual_params = LaunchConfiguration('visual_params',
        default=os.path.join(pkg_dir, 'config/ur5e/visual_parameters.yaml'))
    transmission_hw_interface = LaunchConfiguration('transmission_hw_interface',
        default='hardware_interface/PositionJointInterface')
    safety_limits = LaunchConfiguration('safety_limits', default='false')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin', default='0.15')
    safety_k_position = LaunchConfiguration('safety_k_position', default='20')

    # Include the common launch file
    load_ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'load_ur.launch.py')
        ),
        launch_arguments={
            'joint_limit_params': joint_limit_params,
            'kinematics_params': kinematics_params,
            'physical_params': physical_params,
            'visual_params': visual_params,
            'transmission_hw_interface': transmission_hw_interface,
            'safety_limits': safety_limits,
            'safety_pos_margin': safety_pos_margin,
            'safety_k_position': safety_k_position
        }.items()
    )

    return LaunchDescription([
        load_ur_launch
    ])
