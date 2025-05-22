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

def generate_launch_description():
    pkg_dir = get_package_share_directory('dual_ur_inspire_description')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_dir, 'config', 'view_dual_arm.rviz')

    gui_arg = DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )

    limited_arg = DeclareLaunchArgument(
        'limited',
        default_value='false',
        description='If true, limits joint range [-PI, PI] on all joints.'
    )

    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='If true, use ignite gazebo hardware.'
    )
    
    def launch_setup(context, *args, **kwargs):
        # Evaluate LaunchConfigurations
        limited = LaunchConfiguration('limited').perform(context)
        use_gazebo = LaunchConfiguration('use_gazebo').perform(context)

        robot_description_content = ParameterValue(
            Command([
                'xacro',
                ' ',
                os.path.join(pkg_dir, 'urdf', 'dual_ur_inspire_control.urdf.xacro'),
                ' limited:=', limited,
                ' use_gazebo:=',use_gazebo
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

        control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        remappings=[
            ("robot_description", "/robot_description"),
            ],
            output="both",
        )

        # robot_controllers = PathJoinSubstitution(
        #     [
        #     FindPackageShare("dual_ur_inspire_description"),
        #     "config",
        #     "dual_arm_inspire_controller.yaml",
        #     ]
        # )
        # control_node = Node(
        #     package="controller_manager",
        #     executable="ros2_control_node",
        #     parameters=[robot_controllers],
        #     remappings=[
        #         ("~/robot_description", "/robot_description"),
        #     ],
        #     output="both",
        # )

        # Initialize Arguments
        gui = LaunchConfiguration("gui")

        # gazebo
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments=[("gz_args", " -r -v 3 empty.sdf")],
            condition=IfCondition(gui),
        )
        gazebo_headless = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
            ),
            launch_arguments=[("gz_args", ["--headless-rendering -s -r -v 3 empty.sdf"])],
            condition=UnlessCondition(gui),
        )

        # Gazebo bridge
        gazebo_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        )

        # gz_spawn_entity = Node(
        #     package="ros_gz_sim",
        #     executable="create",
        #     output="screen",
        #     arguments=[
        #         "-file",
        #         os.path.join(pkg_dir, "urdf", "dual_ur_inspire.sdf"),
        #         "-name",
        #         "dual_arm",
        #     ],
        # )
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

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
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


        # Delay rviz start after `joint_state_broadcaster`
        delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[rviz_node],
            )
        )

        # Delay start of joint_state_broadcaster after `robot_controller`
        # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
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

        return [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            control_node,

            gazebo,
            gazebo_headless,
            gazebo_bridge,
            gz_spawn_entity,

            left_arm_controller_spawner,
            # right_arm_controller_spawner,
            # joint_state_broadcaster_spawner,

            delay_rviz_after_joint_state_broadcaster_spawner,
            delay_joint_state_broadcaster_after_left_controller_spawner,
            delay_joint_state_broadcaster_after_right_controller_spawner,
            delay_joint_state_broadcaster_after_left_hand_controller_spawner,
            delay_joint_state_broadcaster_after_right_hand_controller_spawner
            # rviz_node
        ]

    return LaunchDescription([
        gui_arg,
        limited_arg,
        use_gazebo_arg,
        OpaqueFunction(function=launch_setup)
    ]) 