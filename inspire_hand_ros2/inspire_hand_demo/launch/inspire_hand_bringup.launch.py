#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Inspire Hand (e.g., /dev/ttyUSB0, /dev/ttyACM0)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='2',
        description='Device ID for the Inspire Hand'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate for publishing hand state (Hz)'
    )
    
    enable_diagnostics_arg = DeclareLaunchArgument(
        'enable_diagnostics',
        default_value='false',
        description='Enable diagnostic node (currently disabled)'
    )
    
    enable_publisher_arg = DeclareLaunchArgument(
        'enable_publisher',
        default_value='true',
        description='Enable state publisher node'
    )
    
    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect',
        default_value='true',
        description='Enable automatic reconnection on communication failure'
    )
    
    max_reconnect_attempts_arg = DeclareLaunchArgument(
        'max_reconnect_attempts',
        default_value='5',
        description='Maximum number of reconnection attempts'
    )

    # Service server node - provides the main control services
    service_server_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_bringup',
        name='inspire_hand_service_server',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('device_id'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'max_reconnect_attempts': LaunchConfiguration('max_reconnect_attempts')
        }],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # State publisher node - publishes current hand state
    publisher_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_publisher',
        name='inspire_hand_state_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('device_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'max_reconnect_attempts': LaunchConfiguration('max_reconnect_attempts')
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_publisher'))
    )
    
    # Diagnostics node - monitors hand health and status (disabled for now)
    # diagnostics_node = Node(
    #     package='inspire_hand_demo',
    #     executable='inspire_diagnostics',
    #     name='inspire_hand_diagnostics',
    #     parameters=[{
    #         'serial_port': LaunchConfiguration('serial_port'),
    #         'baudrate': LaunchConfiguration('baudrate'),
    #         'device_id': LaunchConfiguration('device_id'),
    #         'auto_reconnect': LaunchConfiguration('auto_reconnect')
    #     }],
    #     output='screen',
    #     emulate_tty=True,
    #     condition=IfCondition(LaunchConfiguration('enable_diagnostics'))
    # )
    
    # Delayed start for publisher to ensure service server is ready
    delayed_publisher = TimerAction(
        period=2.0,
        actions=[publisher_node]
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        device_id_arg,
        publish_rate_arg,
        enable_diagnostics_arg,
        enable_publisher_arg,
        auto_reconnect_arg,
        max_reconnect_attempts_arg,
        service_server_node,
        delayed_publisher
    ]) 