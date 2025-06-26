#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """
    Launch file for controlling multiple Inspire Hand devices.
    Each device gets its own namespace and set of nodes.
    """
    
    # Device 1 arguments
    device1_port_arg = DeclareLaunchArgument(
        'device1_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Inspire Hand device 1'
    )
    
    device1_id_arg = DeclareLaunchArgument(
        'device1_id',
        default_value='2',
        description='Device ID for Inspire Hand device 1'
    )
    
    # Device 2 arguments
    device2_port_arg = DeclareLaunchArgument(
        'device2_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for Inspire Hand device 2'
    )
    
    device2_id_arg = DeclareLaunchArgument(
        'device2_id',
        default_value='2',
        description='Device ID for Inspire Hand device 2'
    )
    
    # Common arguments
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate for publishing hand state (Hz)'
    )

    # Device 1 nodes
    device1_group = GroupAction([
        PushRosNamespace('inspire_hand_1'),
        Node(
            package='inspire_hand_demo',
            executable='inspire_hand_bringup',
            name='inspire_hand_service_server',
            parameters=[{
                'serial_port': LaunchConfiguration('device1_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'device_id': LaunchConfiguration('device1_id')
            }],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='inspire_hand_demo',
            executable='inspire_hand_publisher',
            name='inspire_hand_state_publisher',
            parameters=[{
                'serial_port': LaunchConfiguration('device1_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'device_id': LaunchConfiguration('device1_id'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }],
            output='screen',
            emulate_tty=True
        )
    ])
    
    # Device 2 nodes
    device2_group = GroupAction([
        PushRosNamespace('inspire_hand_2'),
        Node(
            package='inspire_hand_demo',
            executable='inspire_hand_bringup',
            name='inspire_hand_service_server',
            parameters=[{
                'serial_port': LaunchConfiguration('device2_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'device_id': LaunchConfiguration('device2_id')
            }],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='inspire_hand_demo',
            executable='inspire_hand_publisher',
            name='inspire_hand_state_publisher',
            parameters=[{
                'serial_port': LaunchConfiguration('device2_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'device_id': LaunchConfiguration('device2_id'),
                'publish_rate': LaunchConfiguration('publish_rate')
            }],
            output='screen',
            emulate_tty=True
        )
    ])

    return LaunchDescription([
        device1_port_arg,
        device1_id_arg,
        device2_port_arg,
        device2_id_arg,
        baudrate_arg,
        publish_rate_arg,
        device1_group,
        device2_group
    ]) 