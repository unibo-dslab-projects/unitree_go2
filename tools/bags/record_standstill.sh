#!/usr/bin/env bash
# standstill: gyroscope and accelerometer bias and noise, robot standing with motors on.
# Run on the robot computer, inside tmux.

TYPE=standstill
TOPICS=(/utlidar/cloud /utlidar/imu /utlidar/cloud_base /sportmodestate)
OPTIONAL_TOPICS=()
DURATION=120
INSTRUCTIONS="robot stands (motors on, standing pose), nobody touches it"
NOTES_FIELDS=()

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
record_bag
