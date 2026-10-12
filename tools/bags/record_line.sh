#!/usr/bin/env bash
# line: scale and drift, Point-LIO distance against a taped line of known length.
# Run on the robot computer, inside tmux.

TYPE=line
TOPICS=(/utlidar/cloud /utlidar/imu /sportmodestate)
OPTIONAL_TOPICS=(/utlidar/robot_odom)
DURATION=manual
MAX_MINUTES=2
INSTRUCTIONS="walk straight along the taped line and stop on its end"
NOTES_FIELDS=("Line length (m)")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag
