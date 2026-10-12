#!/usr/bin/env bash
# lying: bias without leg-motor vibration, to compare with standstill.
# Run on the robot computer, inside tmux.

TYPE=lying
TOPICS=(/utlidar/cloud /utlidar/imu /utlidar/cloud_base /sportmodestate)
OPTIONAL_TOPICS=()
DURATION=60
INSTRUCTIONS="robot lies down with motors relaxed, nobody touches it"
NOTES_FIELDS=()

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag
