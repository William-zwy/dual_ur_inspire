import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
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
            "use_gazebo",
            default_value='true',
            description='If true, use ignite gazebo hardware.'
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

def launch_setup(context, *args, **kwargs):
    
    rviz_config = context.launch_configurations['rviz_config']
    use_gazebo = context.launch_configurations['use_gazebo']
    
    # Get the package share directory
    moveit_config_pkg = get_package_share_directory("dual_ur_inspire_moveit_config")
    
    moveit_config = (
        MoveItConfigsBuilder("dual_ur_inspire")
        .robot_description(file_path="config/dual_ur_inspire.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_semantic(file_path=os.path.join(moveit_config_pkg, "config", "dual_ur_inspire.srdf"))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),{"use_sim_time": True}],
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
        parameters=[moveit_config.robot_description,{"use_sim_time": True}],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_ur_inspire_description"),
        "config",
        "dual_arm_inspire_controller.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
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
    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["inspire_hand_left_ros2_controller", "-c", "/controller_manager"],
    )
    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["inspire_hand_right_ros2_controller", "-c", "/controller_manager"],
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
        condition=IfCondition(use_gazebo),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
        condition=UnlessCondition(use_gazebo),
    )

    # Gazebo bridge
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "dual_arm",
        ],
    )

    delay_joint_state_broadcaster_after_left_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_after_right_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_hand_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_after_left_hand_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_hand_controller_spawner,
            on_exit=[right_hand_controller_spawner],
        )
    )

    delay_joint_state_broadcaster_after_right_hand_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_hand_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes_to_start = [
        rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,

        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,

        left_arm_controller_spawner,
        delay_joint_state_broadcaster_after_left_controller_spawner,
        delay_joint_state_broadcaster_after_right_controller_spawner,
        delay_joint_state_broadcaster_after_left_hand_controller_spawner,
        delay_joint_state_broadcaster_after_right_hand_controller_spawner
    ]

    return nodes_to_start