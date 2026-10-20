#!/usr/bin/env bash
# Copies bags from the robot to the laptop, then checks that every message arrived: it counts the
# messages in the .db3 files and compares them with metadata.yaml. Run on the laptop, outside the
# container (it needs only ssh, rsync and python3).
#
#   tools/bags/fetch.sh 2026-10-14_loop_01 [more bags...]
#   tools/bags/fetch.sh '2026-10-14_*'               quoted, so the robot expands the pattern
#   tools/bags/fetch.sh --check <local bag folder>...  only checks bags already on the laptop
#
# Environment overrides: ROBOT (default go2), REMOTE_BAG_DIR (default bags, in the robot home),
# LOCAL_BAG_DIR (default: the bags folder next to this repository).

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${ROBOT:-go2}"
REMOTE_BAG_DIR="${REMOTE_BAG_DIR:-bags}"
LOCAL_BAG_DIR="${LOCAL_BAG_DIR:-$(cd "$script_dir/../../.." && pwd)/bags}"

usage() {
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
}

# Prints one line per topic; returns 1 when a count differs or metadata.yaml is missing.
check_local_bag() {
    python3 -I - "$1" <<'PY'
import pathlib
import re
import sqlite3
import sys

bag = pathlib.Path(sys.argv[1])
meta = bag / "metadata.yaml"
print(f"{bag.name}:")
if not meta.is_file():
    print("  no metadata.yaml: still recording, or the recorder was killed. Foxy cannot play it.")
    sys.exit(1)
files = sorted(bag.glob("*.db3"))
if not files:
    print("  no .db3 file")
    sys.exit(1)

expected = {name: int(count) for name, count in
            re.findall(r"name:\s*(\S+).*?message_count:\s*(\d+)", meta.read_text(), re.S)}
if not expected:
    print("  metadata.yaml lists no topics (empty or broken file). Foxy cannot play it.")
    sys.exit(1)
found = {}
for db in files:
    # immutable: read without creating the -shm and -wal files next to the bag
    con = sqlite3.connect(db.resolve().as_uri() + "?mode=ro&immutable=1", uri=True)
    rows = con.execute("select t.name, count(m.id) from topics t "
                       "left join messages m on m.topic_id = t.id group by t.id")
    for name, count in rows:
        found[name] = found.get(name, 0) + count
    con.close()

ok = True
for name in sorted(set(expected) | set(found)):
    want, have = expected.get(name), found.get(name, 0)
    ok &= want == have
    status = "ok" if want == have else f"MISMATCH, metadata.yaml says {want}"
    print(f"  {name:28s} {have:8d}  {status}")
sys.exit(0 if ok else 1)
PY
}

[ $# -ge 1 ] || usage
status=0

if [ "$1" = --check ]; then
    shift
    [ $# -ge 1 ] || usage
    for dir in "$@"; do
        check_local_bag "$dir" || status=1
    done
    exit $status
fi

mkdir -p "$LOCAL_BAG_DIR" || exit 1
names=()
for pattern in "$@"; do
    # ls runs on the robot, so a quoted * works from fish too.
    mapfile -t found < <(ssh "$ROBOT" "cd $REMOTE_BAG_DIR && ls -d -- $pattern 2>/dev/null")
    if [ ${#found[@]} -eq 0 ]; then
        echo "no bag on $ROBOT:$REMOTE_BAG_DIR matches $pattern" >&2
        status=1
    fi
    names+=("${found[@]}")
done

for name in "${names[@]}"; do
    if ! rsync -avP "$ROBOT:$REMOTE_BAG_DIR/$name" "$LOCAL_BAG_DIR/"; then
        echo "rsync failed for $name: run the same command again to resume" >&2
        status=1
        continue
    fi
    if check_local_bag "$LOCAL_BAG_DIR/$name"; then
        echo "  complete. Free the space on the robot with:  ssh $ROBOT rm -r $REMOTE_BAG_DIR/$name"
    else
        status=1
    fi
done
exit $status
