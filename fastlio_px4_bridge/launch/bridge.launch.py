from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value='/home/lingzhilab/ws_livox/src/fastlio_px4_bridge/config/bridge_config.yaml',
            description='Path to bridge node YAML configuration',
        ),

        Node(
            package='fastlio_px4_bridge',
            executable='bridge_node',
            name='fastlio_px4_bridge',
            output='screen',
            parameters=[config_path],
        ),
    ])
