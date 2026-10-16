"""Start the mapping chain: go2_preprocessor, then Point-LIO on its output.

ros2 launch go2_mapping_bringup mapping.launch.py calibration_file:=/abs/path/go2_calibration.yaml
Works the same live and with `ros2 bag play`: both nodes use the stamps inside
the messages, not the clock. (Foxy's bag player has no --clock, so there is no
use_sim_time switch.)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('go2_mapping_bringup'), 'config')

    return LaunchDescription([
        # No default on purpose: without it the preprocessor falls back to
        # CMU's mount values and Point-LIO builds a tilted map.
        DeclareLaunchArgument(
            'calibration_file',
            description='absolute path to the go2_calibration output (go2_calibration.yaml)'),

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
    ])
