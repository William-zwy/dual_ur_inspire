import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the launch arguments
    limited = LaunchConfiguration('limited', default='false')
    transmission_hw_interface = LaunchConfiguration('transmission_hw_interface', default='hardware_interface/PositionJointInterface')

    # Get the package share directory
    pkg_share = get_package_share_directory('dual_ur_inspire_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'dual_ur_robotiq.urdf.xacro')
    
    # Process the URDF file
    robot_description = Command([
        'xacro ', urdf_file, ' transmission_hw_interface:=', transmission_hw_interface
    ])
    robot_description_param = {'robot_description': ParameterValue(robot_description, value_type=str)}

    # Include the dual arm upload launch file
    dual_arm_upload = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'dual_arm_upload.launch.py')]),
        launch_arguments={
            'limited': limited,
            'transmission_hw_interface': transmission_hw_interface,
        }.items(),
    )

    # Launch the joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )

    # Launch RViz2
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        dual_arm_upload,
        joint_state_publisher,
        rviz,
    ])
