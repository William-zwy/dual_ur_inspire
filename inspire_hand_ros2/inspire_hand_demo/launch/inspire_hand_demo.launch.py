#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Demo launch file for Inspire Hand with example client usage.
    Launches the hand control system and optionally runs demo sequences.
    """
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
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
    
    run_demo_arg = DeclareLaunchArgument(
        'run_demo',
        default_value='false',
        description='Run demo sequence after launching'
    )
    
    demo_type_arg = DeclareLaunchArgument(
        'demo_type',
        default_value='basic',
        description='Type of demo to run: basic, grasp, individual_fingers'
    )

    # Main service server node
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
    
    # State publisher node
    publisher_node = Node(
        package='inspire_hand_demo',
        executable='inspire_hand_publisher',
        name='inspire_hand_state_publisher',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'device_id': LaunchConfiguration('device_id'),
            'publish_rate': 5.0
        }],
        output='screen',
        emulate_tty=True
    )
    
    # Demo client node
    demo_client_node = Node(
        package='inspire_hand_demo',
        executable='inspire_client',
        name='inspire_hand_demo_client',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'demo_type': LaunchConfiguration('demo_type')
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('run_demo'))
    )
    
    # Delayed start for publisher and demo client
    delayed_publisher = TimerAction(
        period=2.0,
        actions=[publisher_node]
    )
    
    delayed_demo = TimerAction(
        period=5.0,
        actions=[demo_client_node]
    )
    
    # Command examples that can be run manually
    example_commands_info = ExecuteProcess(
        cmd=['bash', '-c', '''
echo "=== Inspire Hand Demo Launch ==="
echo "The hand control system is starting..."
echo ""
echo "Available manual commands after system starts:"
echo ""
echo "1. Set all fingers to open position:"
echo "   ros2 service call /inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \\"{id: 1, angle1: 0, angle2: 0, angle3: 0, angle4: 0, angle5: 0, angle6: 0}\\""
echo ""
echo "2. Set all fingers to closed position:"
echo "   ros2 service call /inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \\"{id: 1, angle1: 1000, angle2: 1000, angle3: 1000, angle4: 1000, angle5: 1000, angle6: 1000}\\""
echo ""
echo "3. Control individual finger (finger 1 to position 500):"
echo "   ros2 service call /inspire_hand_set_angle_srv inspire_hand_interfaces/srv/SetAngle \\"{id: 1, angle1: 500, angle2: -1, angle3: -1, angle4: -1, angle5: -1, angle6: -1}\\""
echo ""
echo "4. Set finger speeds:"
echo "   ros2 service call /inspire_hand_set_speed_srv inspire_hand_interfaces/srv/SetSpeed \\"{id: 1, speed1: 200, speed2: 200, speed3: 200, speed4: 200, speed5: 200, speed6: 200}\\""
echo ""
echo "5. Monitor hand state:"
echo "   ros2 topic echo /inspire_hand_angle_state"
echo ""
echo "================================"
        '''],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        device_id_arg,
        run_demo_arg,
        demo_type_arg,
        example_commands_info,
        service_server_node,
        delayed_publisher,
        delayed_demo
    ]) 