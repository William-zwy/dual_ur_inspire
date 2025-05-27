import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, GroupAction
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
        default_value='false',
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


        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
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

        return [
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            control_node,

            delay_rviz_after_joint_state_broadcaster_spawner,
            left_arm_controller_spawner,
            delay_right_arm_after_left_arm,
            delay_joint_state_broadcaster_after_right_arm
        ]

    return LaunchDescription([
        gui_arg,
        limited_arg,
        use_gazebo_arg,
        OpaqueFunction(function=launch_setup)
    ]) 