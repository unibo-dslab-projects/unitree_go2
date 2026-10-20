#!/usr/bin/env bash
# speeds (optional): effect of speed and vibration on drift, the loop once slow and once fast.
# Run on the robot computer, inside tmux.

TYPE=speeds
TOPICS=(/utlidar/cloud /utlidar/imu /sportmodestate)
OPTIONAL_TOPICS=(/utlidar/robot_odom)
DURATION=manual
MAX_MINUTES=8
INSTRUCTIONS="walk the loop twice, once slow and once fast, stopping on the start mark each time"
NOTES_FIELDS=("Slow speed setting" "Fast speed setting" "Start mark")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag "$@"
