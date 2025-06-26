#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Simple launch file for basic Inspire Hand control.
    Launches only the service server for basic hand control operations.
    """
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Inspire Hand'
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

    # Service server node - provides the main control services
    service_server_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_bringup',
        name='inspire_hand_service_server',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('device_id')
        }],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        device_id_arg,
        service_server_node
    ]) 