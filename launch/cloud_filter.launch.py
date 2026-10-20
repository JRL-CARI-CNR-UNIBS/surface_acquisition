# Copyright 2026 CARI-JRL
# SPDX-License-Identifier: Apache-2.0
# Author: Manuel Beschi <manuel.beschi@unibs.it>

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('surface_acquisition'), 'config', 'cloud_filter.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('input_cloud', default_value='/wrist/depth/color/points',
                              description='Point cloud published by the RealSense driver'),
        DeclareLaunchArgument('output_cloud', default_value='filtered_cloud',
                              description='Filtered cloud, input of node 2'),
        Node(
            package='surface_acquisition',
            executable='cloud_filter_node',
            name='cloud_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            remappings=[
                ('input_cloud', LaunchConfiguration('input_cloud')),
                ('filtered_cloud', LaunchConfiguration('output_cloud')),
            ],
        ),
    ])
