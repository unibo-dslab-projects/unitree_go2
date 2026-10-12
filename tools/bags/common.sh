# Shared logic for the record_<type>.sh scripts. Not meant to be run on its own.
#
# A type script sets these variables, sources this file and calls record_bag:
#   TYPE             scenario name, part of the bag name: <date>_<TYPE>_<NN>
#   TOPICS           array of topics that must exist; the script stops if one is missing
#   OPTIONAL_TOPICS  array of topics recorded when present, skipped with a warning otherwise
#   DURATION         seconds (the recorder stops by itself) or "manual" (stop with Ctrl-C)
#   MAX_MINUTES      manual bags only: longest expected recording, for the disk space check
#   INSTRUCTIONS     what the robot does, shown before and during the recording
#   NOTES_FIELDS     array of things to measure or write down, asked after the recording
#
# Every record_<type>.sh takes one optional argument, a length in seconds: the bag then stops by
# itself after that time, which helps when one person records and drives the robot. Ctrl-C still
# stops it earlier. Without the argument, DURATION applies.
#
# Environment overrides:
#   BAG_DIR                 where bags go (default ~/bags)
#   ROS_SETUP               sourced when ros2 is not on PATH (default ~/unitree_ros2/setup.sh)
#   ALLOW_CLOCK_MISMATCH=1  record even if the computer clock and the sensor clock disagree

if [ -z "${BASH_VERSION:-}" ]; then
    echo "Run this with bash, not sh." >&2
    exit 1
fi

BAG_DIR="${BAG_DIR:-$HOME/bags}"
ROS_SETUP="${ROS_SETUP:-$HOME/unitree_ros2/setup.sh}"
STILL_SECONDS=5         # still at the start and at the end: Point-LIO estimates gravity from the first samples
CLOCK_TOLERANCE_S=5
MIB_PER_MINUTE=150      # walking measured about 110 MiB/min with cloud and IMU only; margin for the extra topics

# Message rates measured on the 2026-09-28 bags, used to spot lost messages after recording.
declare -A EXPECTED_HZ=(
    [/utlidar/cloud]=15.4
    [/utlidar/cloud_base]=15.4
    [/utlidar/cloud_deskewed]=15.4
    [/utlidar/imu]=250
    [/sportmodestate]=296
    [/utlidar/robot_odom]=150
)

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

load_ros() {
    if ! command -v ros2 >/dev/null 2>&1; then
        [ -f "$ROS_SETUP" ] || die "ros2 not found and $ROS_SETUP does not exist (set ROS_SETUP)"
        # shellcheck disable=SC1090
        source "$ROS_SETUP" >/dev/null
    fi
    command -v ros2 >/dev/null 2>&1 || die "ros2 still not found after sourcing $ROS_SETUP"
}

check_not_recording() {
    if pgrep -f 'ros2 bag record' >/dev/null; then
        die "another recorder is running: $(pgrep -af 'ros2 bag record')"
    fi
}

# Fills RECORD_TOPICS with the required topics plus the optional ones that exist.
resolve_topics() {
    local available topic
    available="$(ros2 topic list 2>/dev/null)" || die "ros2 topic list failed"
    RECORD_TOPICS=()
    for topic in "${TOPICS[@]}"; do
        grep -qxF "$topic" <<<"$available" || die "topic $topic not found. Is the robot on and the environment loaded?"
        RECORD_TOPICS+=("$topic")
    done
    for topic in "${OPTIONAL_TOPICS[@]}"; do
        if grep -qxF "$topic" <<<"$available"; then
            RECORD_TOPICS+=("$topic")
        else
            warn "optional topic $topic not found, not recorded"
        fi
    done
}

# The sensor stamps its messages itself; the robot computer clock has been wrong by hours before
# (2026-09-30: 45 h behind), which gives bags the wrong date and name.
check_clock() {
    local sensor_s computer_s diff
    sensor_s="$(PYTHONUNBUFFERED=1 timeout 10 ros2 topic echo /utlidar/imu 2>/dev/null |
        awk '$1 == "sec:" { print $2; exit }')"
    [ -n "$sensor_s" ] || die "no /utlidar/imu message within 10 s"
    computer_s="$(date +%s)"
    diff=$((computer_s - sensor_s))
    if ((${diff#-} > CLOCK_TOLERANCE_S)); then
        warn "computer clock $(date -d "@$computer_s" '+%F %T'), sensor clock $(date -d "@$sensor_s" '+%F %T'): $diff s apart"
        warn "fix it from the laptop:  ssh -t go2 \"sudo date -s @\$(date +%s)\""
        [ "${ALLOW_CLOCK_MISMATCH:-0}" = 1 ] || die "clocks disagree (ALLOW_CLOCK_MISMATCH=1 records anyway)"
    else
        info "clock ok: computer and sensor $diff s apart"
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
        ask_yes_no "Record anyway?" || exit 1
    else
        info "/utlidar/cloud at $rate Hz"
    fi
}

check_disk() {
    local minutes need_mib free_mib
    if [ "$DURATION" = manual ]; then
        minutes="${MAX_MINUTES:?MAX_MINUTES must be set for manual bags}"
    else
        minutes=$(((DURATION + 59) / 60))
    fi
    need_mib=$((minutes * MIB_PER_MINUTE + 200))
    mkdir -p "$BAG_DIR" || die "cannot create $BAG_DIR"
    free_mib="$(df -Pm "$BAG_DIR" | awk 'NR == 2 { print $4 }')"
    ((free_mib >= need_mib)) || die "only $free_mib MiB free in $BAG_DIR, about $need_mib MiB needed"
    info "disk ok: $free_mib MiB free, about $need_mib MiB needed"
}

# <date>_<TYPE>_<NN> with the first free NN, so an existing bag is never overwritten.
next_bag_path() {
    local day n path
    day="$(date +%F)"
    for n in $(seq -w 1 99); do
        path="$BAG_DIR/${day}_${TYPE}_$n"
        if [ ! -e "$path" ]; then
            printf '%s\n' "$path"
            return 0
        fi
    done
    die "no free name for ${day}_${TYPE} (01 to 99 are taken)"
}

# Runs in the background while the recorder runs in the foreground: shows what to do, and for
# fixed-length bags stops the recorder with SIGINT (the same as Ctrl-C) when the time is up.
ticker() {
    local pid_file="$1" start=$SECONDS elapsed message pid
    while :; do
        elapsed=$((SECONDS - start))
        if ((elapsed < STILL_SECONDS)); then
            message="KEEP THE ROBOT STILL"
        elif [ "$DURATION" = manual ]; then
            message="GO: $INSTRUCTIONS. At the end: $STILL_SECONDS s still, then Ctrl-C"
        elif ((MOVING && DURATION - elapsed <= STILL_SECONDS)); then
            message="KEEP THE ROBOT STILL, stops in $((DURATION - elapsed)) s"
        elif ((MOVING)); then
            message="GO: $INSTRUCTIONS. $((DURATION - elapsed)) s left, the last $STILL_SECONDS s still"
        else
            message="recording, $((DURATION - elapsed)) s left"
        fi
        printf '\r\033[K%4d s  %s' "$elapsed" "$message"
        if [ "$DURATION" != manual ] && ((elapsed >= DURATION)); then
            pid="$(cat "$pid_file" 2>/dev/null)"
            [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null
            return 0
        fi
        sleep 1
    done
}

# The recorder runs in the foreground, so Ctrl-C reaches it directly and it writes metadata.yaml
# before exiting. This script traps SIGINT only to survive it and run the checks afterwards.
run_recorder() {
    local pid_file="$BAG_DIR/.record_$$.pid" ticker_pid
    RECORD_LOG="$BAG_DIR/.record_$$.log"
    rm -f "$pid_file"
    START_TIME="$(date '+%F %T %Z')"

    ticker "$pid_file" &
    ticker_pid=$!
    trap ':' INT
    # bash -c writes its PID, then exec turns into the recorder with the same PID.
    bash -c 'echo $$ >"$1"; shift; exec "$@"' _ "$pid_file" \
        ros2 bag record -o "$BAG_PATH" "${RECORD_TOPICS[@]}" >"$RECORD_LOG" 2>&1
    trap - INT
    kill "$ticker_pid" 2>/dev/null
    wait "$ticker_pid" 2>/dev/null
    printf '\n'
    rm -f "$pid_file"

    if [ -d "$BAG_PATH" ]; then
        mv "$RECORD_LOG" "$BAG_PATH/record.log"
        RECORD_LOG="$BAG_PATH/record.log"
    fi
}

# Duration and message counts from metadata.yaml, compared with the expected rates.
check_bag() {
    local meta="$BAG_PATH/metadata.yaml" duration_ns topic count expected min ok=1
    if [ ! -f "$meta" ]; then
        tail -n 20 "$RECORD_LOG" >&2
        die "no metadata.yaml in $BAG_PATH: Foxy cannot play this bag. Record it again."
    fi
    duration_ns="$(awk '/^  duration:/ { d = 1; next } d && /nanoseconds:/ { print $2; exit }' "$meta")"
    BAG_SECONDS="$(awk -v ns="$duration_ns" 'BEGIN { printf "%.1f", ns / 1e9 }')"
    BAG_COUNTS=()
    info "bag $(basename "$BAG_PATH"): $BAG_SECONDS s"
    while read -r topic count; do
        BAG_COUNTS+=("$topic=$count")
        expected="${EXPECTED_HZ[$topic]:-}"
        if [ -z "$expected" ]; then
            printf '  %-26s %8d\n' "$topic" "$count"
            continue
        fi
        # 2 s of slack: the subscriptions start a moment after the recorder.
        min="$(awk -v hz="$expected" -v ns="$duration_ns" 'BEGIN { printf "%d", 0.9 * hz * (ns / 1e9 - 2) }')"
        if ((count < min)); then
            printf '  %-26s %8d  LOW, expected at least %d\n' "$topic" "$count" "$min"
            ok=0
        else
            printf '  %-26s %8d  ok\n' "$topic" "$count"
        fi
    done < <(awk '/^ *name:/ { name = $2 } /^ *message_count:/ && name != "" { print name, $2; name = "" }' "$meta")
    ((ok)) || warn "messages were lost (CPU load, full disk?): consider recording again"
    if [ "$DURATION" != manual ] && awk -v s="$BAG_SECONDS" -v d="$DURATION" 'BEGIN { exit !(s < d - 2) }'; then
        warn "the bag is shorter than the planned $DURATION s (stopped early?)"
    fi
}

bag_count() {
    local entry
    for entry in "${BAG_COUNTS[@]}"; do
        if [ "${entry%%=*}" = "$1" ]; then
            printf '%s' "${entry#*=}"
            return
        fi
    done
    printf -- '-'
}

write_notes() {
    local notes="$BAG_PATH/notes.txt" field answer
    {
        printf 'bag: %s\n' "$(basename "$BAG_PATH")"
        printf 'type: %s\n' "$TYPE"
        printf 'started: %s on %s\n' "$START_TIME" "$(hostname)"
        printf 'duration_s: %s\n' "$BAG_SECONDS"
        printf 'topics: %s\n' "${RECORD_TOPICS[*]}"
        printf 'scenario: %s\n' "$INSTRUCTIONS"
    } >"$notes"
    info ""
    info "Notes for notes.txt (Enter leaves a field empty):"
    PLACE=""
    UNUSUAL=""
    for field in "Place" "${NOTES_FIELDS[@]}" "People walking by, or anything unusual"; do
        read -r -p "  $field: " answer
        printf '%s: %s\n' "$field" "$answer" >>"$notes"
        case "$field" in
            Place) PLACE="$answer" ;;
            People*) UNUSUAL="$answer" ;;
        esac
    done
}

print_summary() {
    local name
    name="$(basename "$BAG_PATH")"
    info ""
    info "Done: $BAG_PATH"
    info "Row for the bag log in docs/bags/README.md:"
    printf '| `%s` | %s | %s s | %s | %s | %s |\n' "$name" "$PLACE" "$BAG_SECONDS" \
        "$(bag_count /utlidar/cloud)" "$(bag_count /utlidar/imu)" "$UNUSUAL"
    info "Copy it on the laptop, outside the container:  tools/bags/fetch.sh $name"
}

# record_bag [seconds]: the type script passes its own arguments.
record_bag() {
    [ -n "${TYPE:-}" ] || die "TYPE is not set"
    [ -n "${DURATION:-}" ] || die "DURATION is not set"
    ((${#TOPICS[@]} > 0)) || die "TOPICS is empty"

    # The types stopped by hand are the ones where the robot moves.
    MOVING=0
    [ "$DURATION" = manual ] && MOVING=1
    (($# <= 1)) || die "usage: $(basename "$0") [seconds]"
    case "${1:-}" in
        "") ;;
        -h | --help)
            if [ "$DURATION" = manual ]; then
                info "usage: $(basename "$0") [seconds]   (without seconds: stop with Ctrl-C)"
            else
                info "usage: $(basename "$0") [seconds]   (without seconds: $DURATION s)"
            fi
            exit 0
            ;;
        *)
            [[ "$1" =~ ^[0-9]+$ ]] || die "the length must be a whole number of seconds, got '$1'"
            # 10#: a leading zero must not turn the number into octal
            DURATION=$((10#$1))
            ((DURATION > 2 * STILL_SECONDS)) ||
                die "the length must be more than $((2 * STILL_SECONDS)) s ($STILL_SECONDS s still at the start and at the end)"
            ;;
    esac

    info "=== $TYPE bag ==="
    info "Scenario: $INSTRUCTIONS"
    if [ "$DURATION" = manual ]; then
        info "Length: you stop it with Ctrl-C (up to about $MAX_MINUTES min)"
    else
        info "Length: $DURATION s, stops by itself (Ctrl-C stops it earlier)"
    fi
    if ((${#NOTES_FIELDS[@]} > 0)); then
        info "Have ready for the notes:"
        printf '  - %s\n' "${NOTES_FIELDS[@]}"
    fi
    info ""

    load_ros
    check_not_recording
    resolve_topics
    check_clock
    check_cloud_rate
    check_disk
    BAG_PATH="$(next_bag_path)" || exit 1

    info ""
    info "Bag:    $BAG_PATH"
    info "Topics: ${RECORD_TOPICS[*]}"
    read -r -p "Press Enter to start recording (Ctrl-C cancels) " _

    run_recorder
    check_bag
    write_notes
    print_summary
}
