#!/usr/bin/env python3
"""Single locator only — no RViz (for use with utils/sensor/pika_pose_monitor)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dist_limit_arg = DeclareLaunchArgument(
        'dist_limit',
        default_value='0.2',
        description='Distance limit for pose filtering',
    )
    angle_limit_arg = DeclareLaunchArgument(
        'angle_limit',
        default_value='0.2',
        description='Angle limit for pose filtering',
    )
    linear_limit_arg = DeclareLaunchArgument(
        'linear_limit',
        default_value='5.0',
        description='Linear velocity limit',
    )
    angular_limit_arg = DeclareLaunchArgument(
        'angular_limit',
        default_value='20.0',
        description='Angular velocity limit',
    )

    pika_single_locator_node = Node(
        package='pika_locator',
        executable='pika_single_locator_node',
        name='pika_single_locator',
        output='screen',
        parameters=[{
            'dist_limit': LaunchConfiguration('dist_limit'),
            'angle_limit': LaunchConfiguration('angle_limit'),
            'linear_limit': LaunchConfiguration('linear_limit'),
            'angular_limit': LaunchConfiguration('angular_limit'),
        }],
    )

    return LaunchDescription([
        dist_limit_arg,
        angle_limit_arg,
        linear_limit_arg,
        angular_limit_arg,
        pika_single_locator_node,
    ])
