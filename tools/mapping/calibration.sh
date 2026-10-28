#!/usr/bin/env bash
# Calibrates the robot and makes the result the calibration map.sh uses.
# Run on the robot computer, inside tmux. The robot moves by itself for about 100 s.
#
#   tools/mapping/calibration.sh
#
# Writes   ~/calibration/<date>_calibration_NN.yaml   only when every check passes
# Links    ~/calibration/current.yaml to it           only then too; a failed run keeps the old link
# Records  ~/bags/<date>_calibration_NN               always, to run the calibration again offline

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

TOPICS=(/utlidar/cloud /utlidar/cloud_base /utlidar/imu /sportmodestate)
# go2_calibration publishes /calibration/phase itself; running it from a bag needs that topic.
BAG_TOPICS=("${TOPICS[@]}" /calibration/phase)
# cloud and cloud_base for about 100 s, about 4 MiB/s, plus margin
NEED_MIB=1000

case "${1:-}" in
    "") ;;
    -h | --help)
        sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *) die "usage: $(basename "$0")   (no arguments)" ;;
esac

load_ros
check_nothing_running
check_topics "${TOPICS[@]}"
check_clock
check_disk "$BAG_DIR" "$NEED_MIB"
mkdir -p "$CALIBRATION_DIR" || die "cannot create $CALIBRATION_DIR"

name="$(next_name calibration)" || exit 1
bag="$BAG_DIR/$name"
output="$CALIBRATION_DIR/$name.yaml"

info ""
info "=== calibration $name ==="
info "The robot stands still for 30 s, turns two full turns each way, then tilts."
info "It needs a flat floor and 1 m free all around. Keep the remote in hand."
info "Ctrl-C at any moment stops the robot."
info ""
info "Result: $output"
info "Bag:    $bag"
read -r -p "Press Enter to start (Ctrl-C cancels) " _

# Ctrl-C reaches go2_calibration, which stops the robot and exits; the script survives it to stop
# the recorder and report.
trap ':' INT
start_recorder "$bag" "${BAG_TOPICS[@]}"
ros2 run go2_calibration go2_calibration --ros-args -p output:="$output"
status=$?
stop_recorder
trap - INT

info ""
if ((status != 0)) || [ ! -f "$output" ]; then
    warn "calibration failed (exit $status): no new file, current.yaml unchanged"
    if [ -L "$CURRENT_CALIBRATION" ]; then
        info "current.yaml still points to $(readlink "$CURRENT_CALIBRATION")"
    fi
    info "The bag is kept: on the laptop, run go2_calibration on it with -p bag:=<bag folder>."
    exit 1
fi

# Relative target: the folder can be copied elsewhere and the link still works.
ln -sfn "$name.yaml" "$CURRENT_CALIBRATION"
info "current.yaml -> $name.yaml"
info ""
info "Copy to the laptop, outside the container:"
info "  rsync -avP go2:$(from_home "$output") ~/Documents/Uni/thesis/calibration/"
info "  tools/bags/fetch.sh $name"
info "Next: tools/mapping/map.sh"
