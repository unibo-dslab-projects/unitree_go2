"""Start the mapping chain: go2_preprocessor, Point-LIO on its output, then
map_builder on Point-LIO's output.

ros2 launch go2_mapping_bringup mapping.launch.py calibration_file:=/abs/path/go2_calibration.yaml
Works the same live and with `ros2 bag play`: all nodes use the stamps inside
the messages, not the clock. (Foxy's bag player has no --clock, so there is no
use_sim_time switch.)
With map_builder_host:=laptop, map_builder is left out here and
laptop.launch.py starts it on the laptop instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('go2_mapping_bringup')
    config = os.path.join(share, 'config')

    return LaunchDescription([
        # No default on purpose: without it the preprocessor falls back to
        # CMU's mount values and Point-LIO builds a tilted map.
        DeclareLaunchArgument(
            'calibration_file',
            description='absolute path to the go2_calibration output (go2_calibration.yaml)'),

        # Must be the same value on both machines, or two map_builders run
        # (the second one stops itself) or none does.
        DeclareLaunchArgument(
            'map_builder_host',
            default_value='robot',
            description='robot: map_builder runs here; laptop: laptop.launch.py runs it'),

        # Point-LIO hangs on Ctrl+C with Fast-RTPS (the Foxy default) and exits
        # cleanly with CycloneDDS, which the robot uses too. A value already
        # set in the shell is kept.
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            EnvironmentVariable('RMW_IMPLEMENTATION', default_value='rmw_cyclonedds_cpp')),

        # The node name must stay go2_preprocessor: both yaml files are keyed
        # by it, and a different name makes ROS ignore them without an error.
        Node(
            package='go2_preprocessor',
            executable='go2_preprocessor',
            name='go2_preprocessor',
            output='screen',
            parameters=[
                os.path.join(config, 'preprocessor.yaml'),
                LaunchConfiguration('calibration_file'),
            ]),

        # Reads /utlidar/transformed_cloud and /utlidar/transformed_imu.
        # Publishes /cloud_registered (scans in the world frame camera_init)
        # and /aft_mapped_to_init (odometry).
        Node(
            package='point_lio_unilidar',
            executable='pointlio_mapping',
            name='point_lio',
            output='screen',
            parameters=[os.path.join(config, 'point_lio.yaml')]),

        # Reads /cloud_registered and /aft_mapped_to_init, builds the voxel map.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'map_builder.launch.py')),
            condition=LaunchConfigurationEquals('map_builder_host', 'robot')),
    ])
