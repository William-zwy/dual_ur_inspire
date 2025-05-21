import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('dual_ur_inspire_description')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'view_dual_arm.rviz')

    limited_arg = DeclareLaunchArgument(
        'limited',
        default_value='false',
        description='If true, limits joint range [-PI, PI] on all joints.'
    )
    
    def launch_setup(context, *args, **kwargs):
        # Evaluate LaunchConfigurations
        limited = LaunchConfiguration('limited').perform(context)

        robot_description_content = ParameterValue(
            Command([
                'xacro',
                ' ',
                os.path.join(pkg_dir, 'urdf', 'dual_ur_inspire_control.urdf.xacro'),
                ' limited:=', limited
            ]),
            value_type=str
        )
        robot_description = {"robot_description": robot_description_content}

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        )
        
        joint_state_publisher_gui_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )

        ros2_controllers_path = os.path.join(
        pkg_dir,
        "config",
        "dual_arm_inspire_controller.yaml",
        )

        ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="both",
        )

        return [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node
        ]

    return LaunchDescription([
        limited_arg,
        OpaqueFunction(function=launch_setup)
    ]) 