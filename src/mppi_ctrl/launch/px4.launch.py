from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('mppi_ctrl')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'fcu_url',
            default_value='/dev/ttyACM1:57600',
            description='FCU connection URL (e.g., /dev/ttyACM1:57600, udp://:14540@localhost:14557)'
        ),
        DeclareLaunchArgument(
            'gcs_url',
            default_value='',
            description='GCS connection URL (optional)'
        ),
        DeclareLaunchArgument(
            'tgt_system',
            default_value='1',
            description='Target system ID'
        ),
        DeclareLaunchArgument(
            'tgt_component',
            default_value='1',
            description='Target component ID'
        ),
        DeclareLaunchArgument(
            'fcu_protocol',
            default_value='v2.0',
            description='FCU protocol version'
        ),
        DeclareLaunchArgument(
            'respawn',
            default_value='false',
            description='Respawn MAVROS node if it crashes'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for MAVROS node'
        ),
        
        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            respawn=LaunchConfiguration('respawn'),
            respawn_delay=2.0,
            parameters=[
                # os.path.join(pkg_share, 'launch', 'px4_pluginlists.yaml'),
                os.path.join(pkg_share, 'launch', 'px4_config.yaml'),
                {
                    'fcu_url': LaunchConfiguration('fcu_url'),
                    'gcs_url': LaunchConfiguration('gcs_url'),
                    'tgt_system': LaunchConfiguration('tgt_system'),
                    'tgt_component': LaunchConfiguration('tgt_component'),
                    'fcu_protocol': LaunchConfiguration('fcu_protocol'),
                }
            ]
        )
    ])