# Mapping stack

The packages that turn the Go2's lidar into a map:

| Package | Does |
|---|---|
| [go2_calibration](../src/go2_calibration/README.md) | measures how the lidar and its IMU sit on the robot, writes `go2_calibration.yaml` |
| `go2_preprocessor` | turns `/utlidar/cloud` and `/utlidar/imu` into the body frame with that file |
| `go2_mapping_bringup` | launch file and settings that start `go2_preprocessor` and Point-LIO together |
| Point-LIO (`src/external/point_lio_unilidar`, from `go2.repos`) | builds the map and the robot's path |

```
robot or bag                go2_preprocessor                     Point-LIO
/utlidar/cloud ──────►  (+ go2_calibration.yaml)  ──► /utlidar/transformed_cloud ──►  /cloud_registered   (each scan, in the map frame camera_init)
/utlidar/imu   ──────►                            ──► /utlidar/transformed_imu   ──►  /aft_mapped_to_init (pose, ~7 kHz)
                                                                                      PCD/scans.pcd       (whole map, written on Ctrl-C)
```

The launch file only starts the two programs and hands each its settings. They find each other
through the topic names: `point_lio.yaml` sets Point-LIO's `common.lid_topic` and `common.imu_topic`
to the preprocessor's output topics.

The same launch file runs live on the robot and on the laptop against a recorded bag: both programs
use the time stamps inside the messages, not the clock. The rest of this file is the plan for the
first test on the real robot, done offline: record on the robot, replay on the laptop.

## The plan

| Where | Step | Time |
|---|---|---|
| laptop | 0. Check the laptop chain on an old bag | 10 min, before going to the lab |
| robot | 1. Update and build | 5 min |
| robot | 2. Calibrate (the robot moves by itself) | 5 min |
| robot | 3. Record the walks | 20 min |
| laptop | 4. Copy everything | 5 min |
| laptop | 5. Replay each walk | 2 min per bag |
| laptop | 6. Read the result and compare with the tape | |
| laptop | 7. Look at the map and the robot box in RViz | |

To take along: the remote, a tape measure, floor tape, a pen.

The place: a flat wall with at least 7 m of free, flat floor in front of it.

## 0. Check the laptop chain (before Monday)

The container needs `ros-foxy-pcl-ros` (installed 2026-10-15) and the Point-LIO source. Both are
already there. In a fresh clone, from the repository root inside the container:

```bash
vcs import src < go2.repos
rosdep install --from-paths src/external --ignore-src -y
```

Build everything (from fish, outside the container):

```bash
podman exec --user nick --workdir "$PWD" foxy bash -c 'source /opt/ros/foxy/setup.bash && source ~/Documents/Uni/thesis/unitree_ros2/cyclonedds_ws/install/local_setup.bash && colcon build'
```

There is no real calibration file yet, so make a hand-made one from the 2026-10-13 numbers. It is
only for this check: replace it with the real one after step 2. Run this in bash (fish has no
`<<'EOF'`), for example in the container: `distrobox enter foxy -- bash`.

```bash
mkdir -p ~/Documents/Uni/thesis/calibration
cat > ~/Documents/Uni/thesis/calibration/handmade_2026-10-13.yaml <<'EOF'
# hand-made from the 2026-10-13 results, NOT a go2_calibration output
go2_preprocessor:
  ros__parameters:
    lidar_to_body:
      rotation: [0.52611, -0.81014, 0.25862, -0.83867, -0.54464, 0.0, 0.14085, -0.2169, -0.96598]
      translation: [0.2822, 0.0, 0.0]
    imu_to_body:
      rotation: [0.52611, -0.81014, 0.25862, -0.83867, -0.54464, 0.0, 0.14085, -0.2169, -0.96598]
    gyro_bias: {x: 0.0028, y: -0.0055, z: 0.0006}
    gyro_leak: {z_to_x: 0.0335, z_to_y: 0.0093}
    range_offset: 0.0
EOF
```

Then replay `2026-10-13_spin_01` (74 s, the robot turns on the spot about four times). Open three
terminals in the container. In each:

```bash
distrobox enter foxy -- bash
cd ~/Documents/Uni/thesis/unitree_go2
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Terminal 1, the mapping chain. Wait until it prints its startup lines:

```bash
ros2 launch go2_mapping_bringup mapping.launch.py calibration_file:=$HOME/Documents/Uni/thesis/calibration/handmade_2026-10-13.yaml
```

Terminal 2, record Point-LIO's poses:

```bash
mkdir -p ~/Documents/Uni/thesis/replays
ros2 bag record -o ~/Documents/Uni/thesis/replays/2026-10-13_spin_01_odom /aft_mapped_to_init
```

Terminal 3, play the bag. It returns to the prompt after about 74 s:

```bash
ros2 bag play ~/Documents/Uni/thesis/bags/2026-10-13_spin_01 --topics /utlidar/cloud /utlidar/imu
```

Then press Ctrl-C in terminal 2, then in terminal 1. Terminal 1 ends with `terminate called without
an active exception` and `process has died`: that is normal (see step 5). In terminal 3, read the
result:

```bash
/usr/bin/python3 - ~/Documents/Uni/thesis/replays/2026-10-13_spin_01_odom <<'EOF'
import glob, math, sqlite3, sys
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
db = glob.glob(sys.argv[1] + '/*.db3')[0]
rows = sqlite3.connect(db).execute('select data from messages order by timestamp').fetchall()
a, b = (deserialize_message(rows[i][0], Odometry).pose.pose for i in (0, -1))
yaw = lambda q: math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
print(f'{len(rows)} poses; end {math.hypot(b.position.x - a.position.x, b.position.y - a.position.y):.2f} m from start, '
      f'height {100 * (b.position.z - a.position.z):+.1f} cm, heading {yaw(b.orientation) - yaw(a.orientation):+.1f} deg')
EOF
```

Expected (2026-10-17): `541461 poses; end 0.25 m from start, height -2.5 cm, heading -14.1 deg`.
The pose count can be half that: Point-LIO sends about 7000 poses a second and the recorder drops
some when the laptop is busy (284072 in a second run, same result). The distance and heading should
stay within a few cm and a few degrees. If that comes out, the laptop side is ready.

To run it again, delete the old pose recording first: `ros2 bag record` refuses a folder that
already exists.

## 1. On the robot: update and build

From the laptop, set the robot clock first (it falls back to late September after a reboot, which
gives the bags wrong names), copy the recording scripts, and log in:

```bash
ssh -t go2 "sudo date -s @$(date +%s)"
tools/bags/push.sh
ssh go2
```

On the robot:

```bash
cd ~/unitree_go2
git fetch
git checkout feat/mapping-stack
git pull
source ~/unitree_ros2/setup.sh
colcon build --packages-select go2_calibration
source install/setup.bash
```

Offline testing needs only `go2_calibration` on the robot. Point-LIO and the preprocessor run on
the laptop.

## 2. Calibrate

Read the safety part of [go2_calibration](../src/go2_calibration/README.md) first. This is its first live
run: keep the remote in hand. The robot stands still for 30 s, turns two full turns each way, then
tilts, about 97 s in all. It needs a flat floor and 1 m free all around.

Everything in this step runs on the robot computer, typed in the `ssh go2` session from step 1:
the lidar topics exist only on the robot's internal network, and the program steers the robot.
Start `tmux new -s calib` and open two windows (tmux keeps the run alive if the Wi-Fi drops). In both, run `source ~/unitree_ros2/setup.sh` and
`source ~/unitree_go2/install/setup.bash`.

Window 1, the raw data, so the calibration can be run again later from the bag:

```bash
ros2 bag record -o ~/bags/$(date +%F)_calibration_01 \
    /utlidar/cloud /utlidar/cloud_base /utlidar/imu /sportmodestate /calibration/phase
```

Window 2, the calibration itself:

```bash
mkdir -p ~/calibration
ros2 run go2_calibration go2_calibration --ros-args -p output:=$HOME/calibration/$(date +%F)_go2_calibration.yaml
```

When it has finished, stop window 1 with one Ctrl-C.

- Every line of the report must say PASS. Then the file exists. Write down `lidar_to_body`,
  `range_offset` and the floor tilt next to the 2026-10-13 numbers in the go2_calibration README.
- On a FAIL, no file is written. The bag is still useful: keep it and run the calibration from it
  on the laptop (`-p bag:=<bag folder>`). Do not walk with the hand-made file instead: the replays
  would then test the hand-made numbers, not the robot.

## 3. Record the walks

Each walk uses `record_line.sh` (see [tools/bags](../tools/bags/README.md)). The script checks the
topics, the clock and the cloud rate, records, says KEEP THE ROBOT STILL for the first 5 s, and asks
for notes at the end. Point-LIO needs the still start: it builds its first map then (about 0.5 s).

Set up once:

1. Put the robot facing the wall, about 6 m away. Mark the floor under one fixed point of the robot
   (for example the front feet) with tape: that is the start mark.
2. Measure the distance from the start mark to the wall, along the floor.

**Walk A, normal speed.** Run `~/tools/bags/record_line.sh`. Press Enter, keep the robot still for
5 s, walk straight at the wall, stop about 1.5 m before it, keep still for 5 s, press Ctrl-C once.
Mark the floor under the same point of the robot (the end mark). Measure the end mark to the wall
and the start mark to the end mark. Type the start-to-end distance as the line length, and the two
wall distances in the notes.

**Walk B, slow (about 0.3 m/s).** Back to the start mark and repeat walk A. If the distance comes out
wrong in A but right in B, the error comes from speed.

**Walk C, optional: a first room map.** `~/tools/bags/record_loop.sh`: walk a loop through the
room, at normal speed, and end on the start mark. A good map ends where it started.

Write anything unusual (people walking past, a door opened, the robot slipped) in the notes.

## 4. Copy everything to the laptop

On the laptop, outside the container, from the repository root:

```bash
tools/bags/fetch.sh '<date>_*'
scp go2:calibration/<date>_go2_calibration.yaml ~/Documents/Uni/thesis/calibration/
```

`<date>` is the robot's date, for example `2026-10-19`. `fetch.sh` checks every bag and prints the
command to free the space on the robot. It never deletes anything itself.

## 5. Replay a walk

Three terminals in the container. In each:

```bash
distrobox enter foxy -- bash
cd ~/Documents/Uni/thesis/unitree_go2
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`ROS_LOCALHOST_ONLY=1` and domain 77 keep the replay away from the robot if it is on the same
network. CycloneDDS in all three terminals: with Fast-RTPS, Point-LIO hangs on Ctrl-C, and replays
in the container have received nothing.

Terminal 1, the mapping chain:

```bash
ros2 launch go2_mapping_bringup mapping.launch.py calibration_file:=$HOME/Documents/Uni/thesis/calibration/<date>_go2_calibration.yaml
```

Terminal 2, record Point-LIO's poses (the folder must not exist yet):

```bash
mkdir -p ~/Documents/Uni/thesis/replays
ros2 bag record -o ~/Documents/Uni/thesis/replays/<bag>_odom /aft_mapped_to_init
```

Terminal 3, the bag. Only the lidar topics are needed:

```bash
ros2 bag play ~/Documents/Uni/thesis/bags/<bag> --topics /utlidar/cloud /utlidar/imu
```

When the player has finished, press Ctrl-C in terminal 2, then in terminal 1. Point-LIO writes the
map, then exits with `terminate called without an active exception` and launch reports `process
has died`. That is a known flaw of Point-LIO's exit, and the map is already saved. Each run
overwrites the map, so copy it at once:

```bash
cp src/external/point_lio_unilidar/PCD/scans.pcd ~/Documents/Uni/thesis/replays/<bag>.pcd
```

The result depends on the laptop keeping up. If other programs keep the CPU busy (check `uptime`;
the 2026-10-16 replays ran at a load below 2), replay at a quarter of the speed with `-r 0.25`, which
gives the same result every time.

## 6. Read the result

In any of the three terminals. Use `/usr/bin/python3`: inside `distrobox enter`, a bare `python3` is
mise's Python 3.12, which cannot load ROS Foxy's message code.

```bash
/usr/bin/python3 - ~/Documents/Uni/thesis/replays/<bag>_odom <<'EOF'
import glob, math, sqlite3, sys
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
db = glob.glob(sys.argv[1] + '/*.db3')[0]
rows = sqlite3.connect(db).execute('select data from messages order by timestamp').fetchall()
a, b = (deserialize_message(rows[i][0], Odometry).pose.pose for i in (0, -1))
yaw = lambda q: math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
print(f'{len(rows)} poses; end {math.hypot(b.position.x - a.position.x, b.position.y - a.position.y):.2f} m from start, '
      f'height {100 * (b.position.z - a.position.z):+.1f} cm, heading {yaw(b.orientation) - yaw(a.orientation):+.1f} deg')
EOF
```

What to compare, for each walk:

| Point-LIO says | Compare with | Good |
|---|---|---|
| end distance from the start | the taped start-to-end distance | within 2 % (10 cm on 5 m) |
| height | 0: the floor is flat | within a few cm |
| heading | the robot's real turn (about 0 for a straight walk) | within a few degrees |
| (walk C) end distance | 0: the loop ends on the start mark | the smaller the better |

The 2026-10-13 line walk, replayed with the hand-made file, ended 5.23 m from the start, against
4.84 m on the tape (not measured carefully) and 5.00 m from aligning the start and end scans. The
wall walks are there to settle that.

## 7. Look at the map and the robot box in RViz

In a fourth container terminal (same four lines as in step 5), during a replay:

```bash
QT_QPA_PLATFORM=xcb rviz2
```

- **The map:** Fixed Frame `camera_init` (type it), add a PointCloud2 display on `/cloud_registered`,
  and set its Decay Time to 100 s so the scans pile up into the map. Walls must be thin and
  straight, not doubled.
- **The robot box:** a second PointCloud2 display on `/utlidar/transformed_cloud`, Fixed Frame
  `body`. `go2_preprocessor` deletes the points inside `robot_box` (the robot's own legs and body).
  Its values in `go2_mapping_bringup/config/preprocessor.yaml` are still CMU's and may not fit our
  mount. Look for leg points close to the robot (they move while it walks), and for real floor or
  wall that is missing. Change the box in that file, then
  `colcon build --packages-select go2_mapping_bringup` and replay again.

If RViz crashes the display when it opens its window, move `~/.cache/mesa_shader_cache` aside and
start it again.

## After this: live on the robot (not tried yet)

The goal is to map live. The launch file is the same; only the data source changes. What it needs
on the robot, from `~/unitree_go2`:

```bash
vcs import src < go2.repos       # without vcs: sudo apt install python3-vcstool
rosdep install --from-paths src/external --ignore-src -y
colcon build --packages-up-to go2_mapping_bringup
source install/setup.bash
ros2 launch go2_mapping_bringup mapping.launch.py calibration_file:=$HOME/calibration/<date>_go2_calibration.yaml
```

Open points before it can work well:

- `rosdep` needs internet on the robot to install `pcl_ros` and the other Point-LIO dependencies.
- The robot's CycloneDDS is bound to `eth0`, the internal lidar network, so the laptop does not see
  its topics over Wi-Fi. Watching live from the laptop needs CycloneDDS bound to both networks.
- `/aft_mapped_to_init` comes at about 7 kHz, too much for Wi-Fi. Record it on the robot instead,
  or watch only `/cloud_registered`.
- Nobody has measured yet whether the robot computer keeps up with Point-LIO while walking.
- The map is written inside the source folder on the robot
  (`src/external/point_lio_unilidar/PCD/scans.pcd`).
