import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_dir = get_package_share_directory('dual_arm_model')
    
    # Path to the RViz configuration file
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg_dir, "rviz", "view.rviz"), 
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_config",
            default_value=os.path.join(pkg_dir, "urdf", "real_bot.xacro"), 
            description="urdf configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):

    rviz_config = context.launch_configurations['rviz_config']
    xacro_config = context.launch_configurations['xacro_config']

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{"robot_description": ParameterValue(
                            Command(["xacro ", xacro_config]),
                            value_type=str),
                     'use_sim_time': False,
                     }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {"use_sim_time": False},
        ],
    )
    
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    nodes_to_start = [
        rviz_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
    ]

    return nodes_to_start