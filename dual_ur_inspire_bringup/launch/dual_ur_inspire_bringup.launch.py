import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription,GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    bringup_pkg = get_package_share_directory('dual_ur_inspire_bringup')
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(bringup_pkg, "rviz", "bot.rviz"), 
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "params_config",
            default_value=os.path.join(bringup_pkg, "config", "params.yaml"), 
            description="parameter configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_config",
            default_value=os.path.join(bringup_pkg, "urdf", "real_bot_limited.urdf"), 
            description="urdf configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "xacro_config",
            default_value=os.path.join(bringup_pkg, "urdf", "real_bot.xacro"), 
            description="urdf configuration file",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    
    rviz_config = context.launch_configurations['rviz_config']
    params_config = context.launch_configurations['params_config']
    urdf_config = context.launch_configurations['urdf_config']
    xacro_config = context.launch_configurations['xacro_config']

    robot_description_content = ParameterValue(
            Command([
                'xacro',
                ' ',
                xacro_config,
            ]),
            value_type=str
        )
    robot_description = {"robot_description": robot_description_content}

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    with open(urdf_config, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False}]
    )

    joint_state_filter_node = Node(
        package="joint_state_filter",
        executable='joint_state_filter',
        output="screen",
    )

    left_arm_node = Node(
        package="arm_node",
        executable='left_arm_node',
        output="screen",
        parameters=[{"urdf_path": urdf_config},
                    {"use_sim_time": True}],
    )

    right_arm_node = Node(
        package="arm_node",
        executable='right_arm_node',
        output="screen",
        parameters=[{"urdf_path": urdf_config},
                    {"use_sim_time": True}],
    )

    fingers_node = Node(
        package="finger_node",
        executable='finger_node',
        name='fingers_node',
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, params_config,{"use_sim_time": True}],
    remappings=[
        ("robot_description", "/robot_description"),
        ],
        output="both",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_left_ros2_controller", "-c", "/controller_manager"],
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_right_ros2_controller", "-c", "/controller_manager"],
    )

    nodes_to_start = [
        rviz_node,
        robot_state_publisher_node,
        joint_state_filter_node,
        fingers_node,
        left_arm_node,
        right_arm_node,
        control_node,
        left_arm_controller_spawner,
        right_arm_controller_spawner,
    ]

    return nodes_to_start