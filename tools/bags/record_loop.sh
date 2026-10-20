#!/usr/bin/env bash
# loop: drift, the gap between the start and end pose of a closed loop.
# Run on the robot computer, inside tmux.

TYPE=loop
TOPICS=(/utlidar/cloud /utlidar/imu /sportmodestate)
OPTIONAL_TOPICS=(/utlidar/robot_odom)
DURATION=manual
MAX_MINUTES=4
INSTRUCTIONS="walk a closed loop and stop exactly on the start mark, facing the same way"
NOTES_FIELDS=("Start mark" "Room dimensions")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag "$@"
