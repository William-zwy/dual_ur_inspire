from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ur_description')
    
    # Declare launch arguments
    joint_limit_params = LaunchConfiguration('joint_limit_params')
    kinematics_params = LaunchConfiguration('kinematics_params')
    physical_params = LaunchConfiguration('physical_params')
    visual_params = LaunchConfiguration('visual_params')
    transmission_hw_interface = LaunchConfiguration('transmission_hw_interface')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command([
                'xacro ',
                os.path.join(pkg_dir, 'urdf', 'dual_ur5e.urdf.xacro'),
                ' joint_limit_params:=', joint_limit_params,
                ' kinematics_params:=', kinematics_params,
                ' physical_params:=', physical_params,
                ' visual_params:=', visual_params,
                ' transmission_hw_interface:=', transmission_hw_interface,
                ' safety_limits:=', safety_limits,
                ' safety_pos_margin:=', safety_pos_margin,
                ' safety_k_position:=', safety_k_position
            ]), value_type=str)
        }]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'joint_limit_params',
            default_value=os.path.join(pkg_dir, 'config/ur5e/joint_limits.yaml'),
            description='YAML file containing the joint limit values'
        ),
        DeclareLaunchArgument(
            'kinematics_params',
            default_value=os.path.join(pkg_dir, 'config/ur5e/default_kinematics.yaml'),
            description='YAML file containing the robot\'s kinematic parameters'
        ),
        DeclareLaunchArgument(
            'physical_params',
            default_value=os.path.join(pkg_dir, 'config/ur5e/physical_parameters.yaml'),
            description='YAML file containing the physical parameters of the robots'
        ),
        DeclareLaunchArgument(
            'visual_params',
            default_value=os.path.join(pkg_dir, 'config/ur5e/visual_parameters.yaml'),
            description='YAML file containing the visual model of the robots'
        ),
        DeclareLaunchArgument(
            'transmission_hw_interface',
            default_value='hardware_interface/PositionJointInterface',
            description='The hardware interface to use'
        ),
        DeclareLaunchArgument(
            'safety_limits',
            default_value='false',
            description='If True, enable the safety limits controller'
        ),
        DeclareLaunchArgument(
            'safety_pos_margin',
            default_value='0.15',
            description='The lower/upper limits in the safety controller'
        ),
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='20',
            description='Used to set k position in the safety controller'
        ),
        # Launch robot state publisher
        robot_state_publisher
    ])
