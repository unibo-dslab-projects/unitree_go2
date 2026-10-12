#!/usr/bin/env bash
# calib: lidar and IMU mount rotation. Run on the robot computer, inside tmux.
# Also records /utlidar/cloud_base (the firmware's own lidar-to-body transform) and the body IMU in
# /sportmodestate, to check our extrinsics against.

TYPE=calib
TOPICS=(/utlidar/cloud /utlidar/imu /utlidar/cloud_base /sportmodestate)
OPTIONAL_TOPICS=(/utlidar/cloud_deskewed /utlidar/robot_odom /tf /tf_static)
DURATION=20
INSTRUCTIONS="robot stands on a flat floor, nose square to a flat wall about 1.5 m away, nobody moves"
NOTES_FIELDS=("Lidar height above the floor (m)" "Lidar to wall distance (m)" "Photo file name")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag
