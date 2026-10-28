#!/usr/bin/env bash
# Maps live with the calibration from calibration.sh: go2_preprocessor, Point-LIO and map_builder
# run until Ctrl-C, then map_builder writes the map. Run on the robot computer, inside tmux.
#
#   tools/mapping/map.sh          map until Ctrl-C
#   tools/mapping/map.sh --bag    also record the raw input to ~/bags/<date>_map_NN, to replay it later
#
# Writes ~/maps/<time of the first scan, UTC>/map.pcd and trajectory.tum.

source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

TOPICS=(/utlidar/cloud /sportmodestate)
# Launch needs about 3 s before Point-LIO reads; it then takes gravity and its first map from the
# next seconds, which must be still.
STILL_SECONDS=10
NEED_MIB=300
# 110 MiB per walking minute measured with cloud and IMU; this leaves room for about 15 minutes
NEED_MIB_WITH_BAG=2000

record_bag=0
case "${1:-}" in
    "") ;;
    --bag) record_bag=1 ;;
    -h | --help)
        sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
    *) die "usage: $(basename "$0") [--bag]" ;;
esac

# Shows when to start walking. Plain lines rather than a status line: the launch output scrolls
# in between.
ticker() {
    info ">>> KEEP THE ROBOT STILL"
    sleep "$STILL_SECONDS"
    info ">>> GO: walk. To stop: keep still 2 s, then press Ctrl-C once and wait"
}

# Points, poses and end position of a saved run. The path starts at 0 0 0, so for a walk that ends
# on its start mark the end distance is the drift. No walked length: the gait sway inflates it
# (6.81 m for the 5.4 m of 2026-10-19_line_01).
summarize() {
    local folder="$1"
    info "map:   $folder/map.pcd, $(awk '/^POINTS/ { print $2; exit }' "$folder/map.pcd") points"
    awk '
        { x = $2; y = $3; z = $4 }
        END { printf "path:  %d poses, ends %.2f m from the start (x %.2f y %.2f z %.2f m)\n", NR, sqrt(x ^ 2 + y ^ 2 + z ^ 2), x, y, z }
    ' "$folder/trajectory.tum"
}

load_ros
[ -e "$CURRENT_CALIBRATION" ] || die "no $CURRENT_CALIBRATION: run calibration.sh first"
calibration="$(readlink -f "$CURRENT_CALIBRATION")"
check_nothing_running
check_topics "${TOPICS[@]}"
check_cloud_rate
if ((record_bag)); then
    check_disk "$BAG_DIR" "$NEED_MIB_WITH_BAG"
    bag="$BAG_DIR/$(next_name map)" || exit 1
fi
check_disk "$MAP_DIR" "$NEED_MIB"

info ""
info "=== mapping ==="
info "Calibration: $calibration ($((($(date +%s) - $(stat -c %Y "$calibration")) / 86400)) days old)"
info "Maps:        $MAP_DIR"
((record_bag)) && info "Bag:         $bag"
read -r -p "Press Enter to start, with the robot standing still (Ctrl-C cancels) " _

# Marks the start, to find the folder map_builder writes on the way out.
started="$(mktemp)"
# Ctrl-C reaches every node; map_builder saves on it. The script survives it to report.
trap ':' INT
((record_bag)) && start_recorder "$bag" "${TOPICS[@]}"
ticker &
ticker_pid=$!
ros2 launch go2_mapping_bringup mapping.launch.py \
    calibration_file:="$calibration" output_dir:="$MAP_DIR"
kill "$ticker_pid" 2>/dev/null
stop_recorder
trap - INT

info ""
folder="$(find "$MAP_DIR" -mindepth 1 -maxdepth 1 -type d -newer "$started" | sort | tail -n 1)"
rm -f "$started"
if [ -z "$folder" ] || [ ! -f "$folder/map.pcd" ]; then
    warn "no map was saved: did Point-LIO send any scan? Look for map_builder's lines in the output above."
    exit 1
fi
summarize "$folder"
info ""
info "Copy to the laptop, outside the container:"
info "  rsync -avP go2:$(from_home "$folder") ~/Documents/Uni/thesis/maps/"
((record_bag)) && info "  tools/bags/fetch.sh $(basename "$bag")"
exit 0
