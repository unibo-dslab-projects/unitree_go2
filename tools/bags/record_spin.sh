#!/usr/bin/env bash
# spin: IMU-to-body rotation (gyro against /sportmodestate), gyro z scale and the z-to-x/y leak;
# Point-LIO under pure rotation. Run on the robot computer, inside tmux.

TYPE=spin
TOPICS=(/utlidar/cloud /utlidar/imu /utlidar/cloud_base /sportmodestate)
OPTIONAL_TOPICS=()
DURATION=manual
MAX_MINUTES=3
INSTRUCTIONS="turn in place slowly: 2 full turns left, 5 s still, 2 full turns right"
NOTES_FIELDS=("Start heading (tape arrow on the floor)")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag "$@"
