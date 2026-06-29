from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odometry_topic', default_value='/vins_estimator/odometry',
            description='VINS Odometry topic to subscribe'),
        DeclareLaunchArgument(
            'body_frame', default_value='FLU',
            description='Body frame convention: FLU (VINS IMU) or FRD'),
        DeclareLaunchArgument(
            'yaw_alignment_mode', default_value='none',
            description="Yaw alignment: 'none', 'px4_mag', or 'manual'"),
        DeclareLaunchArgument(
            'manual_yaw_offset_rad', default_value='0.0',
            description='Fixed yaw offset when yaw_alignment_mode=manual'),
        DeclareLaunchArgument(
            'position_jump_threshold', default_value='0.5',
            description='Position jump threshold in meters'),
        DeclareLaunchArgument(
            'default_position_variance', default_value='[0.01, 0.01, 0.01]',
            description='Default position variance [m^2]'),
        DeclareLaunchArgument(
            'default_orientation_variance', default_value='[0.01, 0.01, 0.01]',
            description='Default orientation variance [rad^2]'),
        DeclareLaunchArgument(
            'default_velocity_variance', default_value='[0.01, 0.01, 0.01]',
            description='Default velocity variance [(m/s)^2]'),

        Node(
            package='vins_px4_bridge',
            executable='bridge_node',
            name='vins_px4_bridge',
            output='screen',
            parameters=[{
                'odometry_topic': LaunchConfiguration('odometry_topic'),
                'body_frame': LaunchConfiguration('body_frame'),
                'yaw_alignment_mode': LaunchConfiguration('yaw_alignment_mode'),
                'manual_yaw_offset_rad': LaunchConfiguration('manual_yaw_offset_rad'),
                'position_jump_threshold': LaunchConfiguration('position_jump_threshold'),
                'default_position_variance': LaunchConfiguration('default_position_variance'),
                'default_orientation_variance': LaunchConfiguration('default_orientation_variance'),
                'default_velocity_variance': LaunchConfiguration('default_velocity_variance'),
            }],
        ),
    ])
