"""Start map_builder alone.

Not meant to be run by hand: mapping.launch.py (robot side) and
laptop.launch.py (laptop side) include it, and their map_builder_host argument
decides which machine runs it.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('go2_mapping_bringup'), 'config')
    with open(os.path.join(config, 'map_builder.yaml')) as file:
        voxel_size = yaml.safe_load(file)['map_builder']['ros__parameters']['voxel_size']

    return LaunchDescription([
        # expanduser runs on the machine that starts the launch, so ~ is the
        # home of the robot or of the laptop, whichever runs map_builder.
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.expanduser('~/maps'),
            description='folder where the save service writes one sub-folder per run'),

        # The node name must stay map_builder: map_builder.yaml is keyed by it.
        # Its inputs are Point-LIO's outputs: the registered scans (frame
        # camera_init) and the odometry.
        Node(
            package='map_builder',
            executable='map_builder',
            name='map_builder',
            output='screen',
            parameters=[
                os.path.join(config, 'map_builder.yaml'),
                {'output_dir': LaunchConfiguration('output_dir')},
            ],
            remappings=[
                ('cloud', '/cloud_registered'),
                ('odom', '/aft_mapped_to_init'),
            ]),

        DeclareLaunchArgument(
            'backend_host',
            default_value='',
            description='address of the laptop running vcortex; empty: no vcortex_bridge'),
        DeclareLaunchArgument(
            'ingest_token',
            default_value=EnvironmentVariable('VCORTEX_INGEST_TOKEN', default_value=''),
            description='same value as VCORTEX_INGEST_TOKEN on the laptop; empty when that is unset'),

        # Next to map_builder, so GetMap's whole map never crosses the Wi-Fi as DDS.
        # voxel_size comes from map_builder.yaml: the backend must use map_builder's grid.
        Node(
            package='vcortex_bridge',
            executable='vcortex_bridge',
            name='vcortex_bridge',
            output='screen',
            parameters=[{
                'backend_host': LaunchConfiguration('backend_host'),
                'token': LaunchConfiguration('ingest_token'),
                'voxel_size': voxel_size,
            }],
            condition=LaunchConfigurationNotEquals('backend_host', '')),
    ])
