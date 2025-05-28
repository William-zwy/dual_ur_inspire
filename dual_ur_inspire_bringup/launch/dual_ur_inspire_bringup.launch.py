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

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="dual_arm_moveit.rviz",
            description="RViz configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'limited',
            default_value='false',
            description='If true, limits joint range [-PI, PI] on all joints.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value='false',
            description='If true, use ignite gazebo hardware.'
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    
    rviz_config = context.launch_configurations['rviz_config']
    limited = context.launch_configurations['limited']
    use_gazebo = context.launch_configurations['use_gazebo']
    
    # Get the package share directory
    moveit_config_pkg = get_package_share_directory("dual_ur_inspire_moveit_config")
    description_pkg = get_package_share_directory('dual_ur_inspire_description')

    robot_description_content = ParameterValue(
        Command([
            'xacro',
            ' ',
            os.path.join(description_pkg, 'urdf', 'dual_ur_inspire_control.urdf.xacro'),
            ' limited:=', limited,
            ' use_gazebo:=',use_gazebo
        ]),
        value_type=str
    )
    full_robot_description = {"robot_description": robot_description_content}
    
    moveit_config = (
        MoveItConfigsBuilder("dual_ur_inspire")
        .robot_description(file_path="config/dual_ur_inspire_moveit.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path=os.path.join(moveit_config_pkg, "config", "dual_ur_inspire_moveit.srdf"))
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl","chomp"]
        )
        .joint_limits(
        file_path=os.path.join(moveit_config_pkg, "config", "joint_limits.yaml")
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"use_sim_time": True},
                    ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description_content},
                    {"use_sim_time": True}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_ur_inspire_description"),
        "config",
        "dual_arm_inspire_controller.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[full_robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_filter_node = Node(
        package="moveit_node",
        executable='joint_state_filter',
        name='filtered_joint_state_publisher',
        output="screen",
    )

    dual_ur_inspire_node = Node(
        package="moveit_node",
        executable='dual_ur_inspire_node',
        name='dual_ur_inspire_node',
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
    )
    
    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_left_ros2_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )
    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_arm_right_ros2_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )



    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of controllers
    delay_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes_to_start = [
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_filter_node,
        dual_ur_inspire_node,

        left_arm_controller_spawner,
        delay_right_arm_after_left_arm,
        delay_joint_state_broadcaster_after_right_arm,
        # delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return nodes_to_start