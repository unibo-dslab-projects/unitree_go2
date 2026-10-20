#!/usr/bin/env bash
# Copies the recording scripts to the robot. Run on the laptop after changing them.
#
# Environment overrides: ROBOT (default go2), REMOTE_TOOLS_DIR (default tools/bags, in the robot home).

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${ROBOT:-go2}"
REMOTE_TOOLS_DIR="${REMOTE_TOOLS_DIR:-tools/bags}"

ssh "$ROBOT" "mkdir -p $REMOTE_TOOLS_DIR" || exit 1
# fetch.sh and push.sh run on the laptop only.
rsync -av --exclude fetch.sh --exclude push.sh "$script_dir/" "$ROBOT:$REMOTE_TOOLS_DIR/"
