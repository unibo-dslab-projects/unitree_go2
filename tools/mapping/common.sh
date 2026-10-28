# Shared logic for calibration.sh and map.sh. Not meant to be run on its own.
#
# Environment overrides:
#   ROS_SETUP        ROS 2 with the Unitree messages, always sourced (default ~/unitree_ros2/setup.sh)
#   WS_SETUP         this workspace (default install/setup.bash at the repository root)
#   CALIBRATION_DIR  calibration files and the current.yaml link (default ~/calibration)
#   BAG_DIR          bags (default ~/bags)
#   MAP_DIR          maps, one folder per run (default ~/maps)

if [ -z "${BASH_VERSION:-}" ]; then
    echo "Run this with bash, not sh." >&2
    exit 1
fi

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-$HOME/unitree_ros2/setup.sh}"
WS_SETUP="${WS_SETUP:-$REPO_DIR/install/setup.bash}"
CALIBRATION_DIR="${CALIBRATION_DIR:-$HOME/calibration}"
CURRENT_CALIBRATION="$CALIBRATION_DIR/current.yaml"
BAG_DIR="${BAG_DIR:-$HOME/bags}"
MAP_DIR="${MAP_DIR:-$HOME/maps}"
CLOCK_TOLERANCE_S=5

info() { printf '%s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }
die() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

ask_yes_no() {
    local answer
    read -r -p "$1 [y/N] " answer
    [[ "$answer" == [yY]* ]]
}

# The robot's ~/.bashrc puts ros2 on PATH without the unitree_go messages, so ROS_SETUP is always
# sourced: without it /sportmodestate cannot be read or recorded.
load_ros() {
    [ -f "$ROS_SETUP" ] || die "$ROS_SETUP does not exist (set ROS_SETUP)"
    # shellcheck disable=SC1090
    source "$ROS_SETUP" >/dev/null
    [ -f "$WS_SETUP" ] || die "$WS_SETUP does not exist: build the workspace in $REPO_DIR first"
    # shellcheck disable=SC1090
    source "$WS_SETUP"
    command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $ROS_SETUP"
}

# A second preprocessor or Point-LIO publishes on the same topics and mixes two maps into one, and
# the calibration steers the robot while it runs.
check_nothing_running() {
    local running
    running="$(pgrep -af '(^|/)(go2_calibration|go2_preprocessor|pointlio_mapping|map_builder)( |$)')"
    [ -z "$running" ] || die "already running, stop it first (tmux ls shows the old session):"$'\n'"$running"
}

check_topics() {
    local available topic
    available="$(ros2 topic list 2>/dev/null)" || die "ros2 topic list failed"
    for topic in "$@"; do
        grep -qxF "$topic" <<<"$available" || die "topic $topic not found. Is the robot on?"
    done
}

# The sensor stamps its messages itself; the robot computer clock falls back to late September
# after a reboot, and the date names the calibration files and bags.
check_clock() {
    local sensor_s computer_s diff
    sensor_s="$(PYTHONUNBUFFERED=1 timeout 10 ros2 topic echo /utlidar/imu 2>/dev/null |
        awk '$1 == "sec:" { print $2; exit }')"
    [ -n "$sensor_s" ] || die "no /utlidar/imu message within 10 s"
    computer_s="$(date +%s)"
    diff=$((computer_s - sensor_s))
    if ((${diff#-} > CLOCK_TOLERANCE_S)); then
        warn "computer clock $(date -d "@$computer_s" '+%F %T'), sensor clock $(date -d "@$sensor_s" '+%F %T'): $diff s apart"
        warn "the file names will carry the wrong date. Fix it from the laptop:  ssh -t go2 \"sudo date -s @\$(date +%s)\""
        ask_yes_no "Go on anyway?" || exit 1
    fi
}

check_cloud_rate() {
    local rate
    info "measuring the /utlidar/cloud rate for 8 s..."
    rate="$(PYTHONUNBUFFERED=1 timeout -s INT 8 ros2 topic hz /utlidar/cloud 2>/dev/null |
        awk '/average rate:/ { r = $3 } END { print r }')"
    [ -n "$rate" ] || die "no /utlidar/cloud messages within 8 s"
    if awk -v r="$rate" 'BEGIN { exit !(r < 14 || r > 17) }'; then
        warn "/utlidar/cloud at $rate Hz, expected about 15.4 Hz: something is using the CPU or the lidar is struggling"
        ask_yes_no "Go on anyway?" || exit 1
    else
        info "/utlidar/cloud at $rate Hz"
    fi
}

# check_disk <dir> <MiB needed>
check_disk() {
    local free_mib
    mkdir -p "$1" || die "cannot create $1"
    free_mib="$(df -Pm "$1" | awk 'NR == 2 { print $4 }')"
    ((free_mib >= $2)) || die "only $free_mib MiB free in $1, about $2 MiB needed"
}

# next_name <type>: <date>_<type>_<NN> with the first NN free in BAG_DIR and CALIBRATION_DIR, so the
# bag and the calibration file of one run share their name and nothing is overwritten.
next_name() {
    local day n name
    day="$(date +%F)"
    for n in $(seq -w 1 99); do
        name="${day}_$1_$n"
        if [ ! -e "$BAG_DIR/$name" ] && [ ! -e "$CALIBRATION_DIR/$name.yaml" ]; then
            printf '%s\n' "$name"
            return 0
        fi
    done
    die "no free name for ${day}_$1 (01 to 99 are taken)"
}

# start_recorder <bag path> <topic>...: records in the background. A Ctrl-C in the terminal reaches
# the recorder too (rclcpp installs its own SIGINT handler), and it writes metadata.yaml on its way
# out; stop_recorder covers the runs that end without one.
start_recorder() {
    local bag="$1"
    shift
    RECORDER_LOG="$BAG_DIR/.record_$$.log"
    ros2 bag record -o "$bag" "$@" >"$RECORDER_LOG" 2>&1 &
    RECORDER_PID=$!
    RECORDER_BAG="$bag"
}

stop_recorder() {
    [ -n "${RECORDER_PID:-}" ] || return 0
    kill -INT "$RECORDER_PID" 2>/dev/null
    for _ in $(seq 20); do
        kill -0 "$RECORDER_PID" 2>/dev/null || break
        sleep 0.5
    done
    if kill -0 "$RECORDER_PID" 2>/dev/null; then
        warn "the recorder did not stop within 10 s; killing it"
        kill -KILL "$RECORDER_PID" 2>/dev/null
    fi
    wait "$RECORDER_PID" 2>/dev/null
    RECORDER_PID=""
    if [ ! -d "$RECORDER_BAG" ]; then
        warn "no bag was written, see $RECORDER_LOG"
        return 0
    fi
    mv "$RECORDER_LOG" "$RECORDER_BAG/record.log"
    if [ ! -f "$RECORDER_BAG/metadata.yaml" ]; then
        warn "no metadata.yaml in $RECORDER_BAG: Foxy cannot play this bag (log: $RECORDER_BAG/record.log)"
    fi
}

# Path relative to the home folder, for the copy commands printed for the laptop.
from_home() {
    printf '%s\n' "${1#"$HOME"/}"
}
