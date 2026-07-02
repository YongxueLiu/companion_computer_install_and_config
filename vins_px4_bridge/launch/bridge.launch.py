from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():
    default_config = join(
        get_package_share_directory('vins_px4_bridge'),
        'config',
        'bridge_config.yaml',
    )
    config_path = LaunchConfiguration('config', default=default_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to bridge node YAML configuration',
        ),

        Node(
            package='vins_px4_bridge',
            executable='bridge_node',
            name='vins_px4_bridge',
            output='screen',
            parameters=[config_path],
        ),
    ])
