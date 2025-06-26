#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription, 
    DeclareLaunchArgument,
    LogInfo,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    left_hand_port_arg = DeclareLaunchArgument(
        'left_hand_port',
        default_value='/dev/ttyACM0',
        description='Serial port for left inspire hand'
    )
    
    right_hand_port_arg = DeclareLaunchArgument(
        'right_hand_port', 
        default_value='/dev/ttyUSB1',
        description='Serial port for right inspire hand'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial communication baud rate'
    )
    
    left_device_id_arg = DeclareLaunchArgument(
        'left_device_id',
        default_value='2',
        description='Device ID for left hand'
    )
    
    right_device_id_arg = DeclareLaunchArgument(
        'right_device_id',
        default_value='1', 
        description='Device ID for right hand'
    )
    
    # Left hand node with namespace
    left_hand_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_bringup',
        name='inspire_hand_service_server',
        namespace='inspire_hand_left',
        parameters=[{
            'serial_port': LaunchConfiguration('left_hand_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('left_device_id'),
            'auto_reconnect': True,
            'max_reconnect_attempts': 5,
            'publish_rate': 10.0
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Right hand node with namespace
    right_hand_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_bringup', 
        name='inspire_hand_service_server',
        namespace='inspire_hand_right',
        parameters=[{
            'serial_port': LaunchConfiguration('right_hand_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('right_device_id'),
            'auto_reconnect': True,
            'max_reconnect_attempts': 5,
            'publish_rate': 10.0
        }],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Delay right hand startup to avoid serial conflicts
    delayed_right_hand = TimerAction(
        period=3.0,
        actions=[right_hand_node]
    )
    
    # Information messages
    info_msg = LogInfo(
        msg=[
            '\n=== Inspire Hand Dual Setup ===\n',
            'Left Hand Port: ', LaunchConfiguration('left_hand_port'), '\n',
            'Right Hand Port: ', LaunchConfiguration('right_hand_port'), '\n',
            'Baudrate: ', LaunchConfiguration('baudrate'), '\n',
            'Left Device ID: ', LaunchConfiguration('left_device_id'), '\n',
            'Right Device ID: ', LaunchConfiguration('right_device_id'), '\n',
            '================================\n'
        ]
    )
    
    service_info = LogInfo(
        msg=[
            '\n=== Available Services ===\n',
            'Left Hand Services:\n',
            '  /inspire_hand_left/inspire_hand_set_angle_srv\n',
            '  /inspire_hand_left/inspire_hand_set_speed_srv\n', 
            '  /inspire_hand_left/inspire_hand_set_force_srv\n',
            'Right Hand Services:\n',
            '  /inspire_hand_right/inspire_hand_set_angle_srv\n',
            '  /inspire_hand_right/inspire_hand_set_speed_srv\n',
            '  /inspire_hand_right/inspire_hand_set_force_srv\n',
            '==========================\n'
        ]
    )
    
    # Delayed service info
    delayed_service_info = TimerAction(
        period=5.0,
        actions=[service_info]
    )
    
    return LaunchDescription([
        left_hand_port_arg,
        right_hand_port_arg,
        baudrate_arg,
        left_device_id_arg,
        right_device_id_arg,
        info_msg,
        left_hand_node,
        delayed_right_hand,
        delayed_service_info
    ]) 