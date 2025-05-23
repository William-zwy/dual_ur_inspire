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
            default_value='true',
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
            pipelines=["ompl"]
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

    # Left hand joint controllers
    left_thumb_yaw_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_thumb_proximal_yaw_joint_controller", "-c", "/controller_manager"],
    )
    left_thumb_pitch_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_thumb_proximal_pitch_joint_controller", "-c", "/controller_manager"],
    )
    left_index_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_index_proximal_joint_controller", "-c", "/controller_manager"],
    )
    left_middle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_middle_proximal_joint_controller", "-c", "/controller_manager"],
    )
    left_ring_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_ring_proximal_joint_controller", "-c", "/controller_manager"],
    )
    left_pinky_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_hand_L_pinky_proximal_joint_controller", "-c", "/controller_manager"],
    )

    # Right hand joint controllers
    right_thumb_yaw_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_thumb_proximal_yaw_joint_controller", "-c", "/controller_manager"],
    )
    right_thumb_pitch_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_thumb_proximal_pitch_joint_controller", "-c", "/controller_manager"],
    )
    right_index_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_index_proximal_joint_controller", "-c", "/controller_manager"],
    )
    right_middle_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_middle_proximal_joint_controller", "-c", "/controller_manager"],
    )
    right_ring_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_ring_proximal_joint_controller", "-c", "/controller_manager"],
    )
    right_pinky_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_hand_R_pinky_proximal_joint_controller", "-c", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Create controller groups
    arm_controllers = GroupAction(
        actions=[
            left_arm_controller_spawner,
            right_arm_controller_spawner,
        ]
    )

    left_hand_controllers = GroupAction(
        actions=[
            left_thumb_yaw_controller_spawner,
            left_thumb_pitch_controller_spawner,
            left_index_controller_spawner,
            left_middle_controller_spawner,
            left_ring_controller_spawner,
            left_pinky_controller_spawner,
        ]
    )

    right_hand_controllers = GroupAction(
        actions=[
            right_thumb_yaw_controller_spawner,
            right_thumb_pitch_controller_spawner,
            right_index_controller_spawner,
            right_middle_controller_spawner,
            right_ring_controller_spawner,
            right_pinky_controller_spawner,
        ]
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of controllers
    delay_left_hand_after_arms = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_hand_controllers],
        )
    )

    delay_right_hand_after_left_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_pinky_controller_spawner,
            on_exit=[right_hand_controllers],
        )
    )

    delay_joint_state_broadcaster_after_hands = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_pinky_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # gazebo
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
    #     condition=IfCondition(use_gazebo),
    # )
    # gazebo_headless = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    #     ),
    #     launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
    #     condition=UnlessCondition(use_gazebo),
    # )
    dual_ur_inspire_description_dir = get_package_share_directory('dual_ur_inspire_description')
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
    ),
    launch_arguments=[("gz_args", [" -r -v 3 " + os.path.join(
        dual_ur_inspire_description_dir,
        "world",
        "ur_dual_inspire_world.sdf"
    )])],
    condition=IfCondition(use_gazebo),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 " + os.path.join(
            dual_ur_inspire_description_dir,
            "world",
            "ur_dual_inspire_world.sdf"
        )])],
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

    nodes_to_start = [
        # rviz_node,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,

        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,

        arm_controllers,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_left_hand_after_arms,
        delay_right_hand_after_left_hand,
        delay_joint_state_broadcaster_after_hands
    ]

    return nodes_to_start