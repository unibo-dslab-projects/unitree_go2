#!/usr/bin/env bash
# floor: the thesis dataset, the whole floor for the map and then the floor plan.
# Run on the robot computer, inside tmux.

TYPE=floor
TOPICS=(/utlidar/cloud /utlidar/imu /sportmodestate)
OPTIONAL_TOPICS=(/utlidar/robot_odom)
DURATION=manual
MAX_MINUTES=20
INSTRUCTIONS="walk every room and corridor that belongs in the plan, back to the start mark"
NOTES_FIELDS=("Rooms and corridors covered" "Room dimensions (tape or laser meter)" "Photo file names")

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag "$@"
