# map_builder

Collects Point-LIO's output into one map and one path, and saves both to disk.

- **In:** `/cloud_registered` (each scan, already placed in the map frame `camera_init`) and
  `/aft_mapped_to_init` (the robot's pose). The launch file connects them to the node's `cloud`
  and `odom` inputs.
- **Map:** a voxel map. Space is cut into cubes of `voxel_size` (5 cm); each cube keeps the first
  point that lands in it and ignores the rest. Point-LIO sends the same wall again with every scan,
  so this removes the repeats.
- **Out, while running:**

| Name | Kind | What |
|---|---|---|
| `/map_builder/delta` | topic, `go2_mapping_msgs/MapDelta` | the cubes that are new since the last delta, 2 times a second |
| `/map_builder/pose` | topic, `geometry_msgs/PoseStamped` | the newest robot pose, 10 times a second |
| `/map_builder/full_map` | topic, `sensor_msgs/PointCloud2` | the whole map; off unless `full_map_rate_hz` > 0 |
| `/map_builder/get_map` | service, `go2_mapping_msgs/GetMap` | the whole map now, plus the `seq` of the last delta inside it |
| `/map_builder/save` | service, `std_srvs/Trigger` | writes `map.pcd` and `trajectory.tum` |

- **Out, on disk** (only when `save` is called): one folder per run inside `output_dir`, named after
  the time of the first scan in UTC, for example `~/maps/2026-10-19T14:35:23/`. Saving twice in the
  same run gives `..._1`, `..._2`.
  - `map.pcd`: one point per cube, fields `x y z intensity`, metres, frame `camera_init`.
  - `trajectory.tum`: one line per pose, `stamp x y z qx qy qz qw` (TUM format, stamp in seconds).

Settings are in `go2_mapping_bringup/config/map_builder.yaml`. That file is copied at build time:
after editing it, run `colcon build --packages-select go2_mapping_bringup`.

## Testing on a recorded bag

This replays the 2026-10-19 straight walk (`2026-10-19_line_01`, taped 5.40 m) through the whole
chain: go2_preprocessor, Point-LIO, map_builder. Everything runs on the laptop, in the `foxy`
container. It takes about 5 minutes.

### 1. Build

From fish, outside the container, at the repository root:

```bash
podman exec --user nick --workdir "$PWD" foxy bash -c 'source /opt/ros/foxy/setup.bash && source ~/Documents/Uni/thesis/unitree_ros2/cyclonedds_ws/install/local_setup.bash && colcon build --packages-up-to go2_mapping_bringup'
```

The `** WARNING ** io features related to openni ...` lines come from PCL and are normal.

### 2. Open three terminals

In each one:

```bash
distrobox enter foxy -- bash
cd ~/Documents/Uni/thesis/unitree_go2
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

`ROS_LOCALHOST_ONLY=1` and domain 77 keep the test away from the robot if it is on the same network.

Before starting, check that nothing from an earlier run is still alive (a closed terminal can leave
nodes running in the container):

```bash
pgrep -af "pointlio_mapping|map_builder|go2_preprocessor"
```

It must print nothing. If it prints something, stop it with `pkill -f <name>`.

### 3. Terminal 1: start the chain

```bash
ros2 launch go2_mapping_bringup mapping.launch.py \
    calibration_file:=$HOME/Documents/Uni/thesis/calibration/2026-10-19_go2_calibration.yaml \
    output_dir:=$HOME/Documents/Uni/thesis/maps
```

Wait for this line from map_builder:

```
[map_builder]: voxel size 0.050 m, delta 2.0 Hz, pose 10.0 Hz, full map 0.0 Hz, run_id ..., output_dir .../maps
```

### 4. Terminal 2: watch the deltas

```bash
PYTHONUNBUFFERED=1 ros2 topic echo --no-arr /map_builder/delta | grep '^seq:'
```

`--no-arr` hides the point data, and `grep '^seq:'` keeps only the counter (a plain `grep seq`
also matches the `<sequence ...>` lines). `PYTHONUNBUFFERED=1` is needed because `ros2 topic echo`
is a Python program: when its output goes into a pipe instead of the screen, Python holds the lines
back and prints them in bursts about every 20 s. Nothing appears until the bag starts.

### 5. Terminal 3: play the bag

```bash
ros2 bag play -r 0.25 ~/Documents/Uni/thesis/bags/2026-10-19_line_01 --topics /utlidar/cloud /utlidar/imu
```

`-r 0.25` plays at a quarter of the speed, so the laptop always keeps up and the result is the same
every time. It returns to the prompt after about 85 s.

While it plays:

- Terminal 1 prints `First scan: frame_id=camera_init, ...` once, then every 2 s
  `Voxels <total>, added <n> points from cloud`. The total only grows.
- Terminal 2 prints `seq: 1`, `seq: 2`, `seq: 3`, ... with no number skipped.

### 6. Terminal 3: save

When the bag has finished:

```bash
ros2 service call /map_builder/save std_srvs/srv/Trigger
```

The answer names the folder:

```
std_srvs.srv.Trigger_Response(success=True, message='/home/nick/Documents/Uni/thesis/maps/2026-10-19T14:35:23')
```

Terminal 1 prints `saved 45195 points and 296 poses to ...`.

Then press Ctrl-C in terminal 2, then in terminal 1. Point-LIO ends with `terminate called without
an active exception` and launch reports `process has died`: that is Point-LIO's known exit flaw
(see `mapping_stack/README.md`, step 5), not map_builder. Point-LIO also writes its own map on
Ctrl-C, over `src/external/point_lio_unilidar/PCD/scans.pcd`: copy that file first if you still
need the old one.

### 7. Check the result

In terminal 3, with the folder from step 6:

```bash
RUN=~/Documents/Uni/thesis/maps/2026-10-19T14:35:23
grep POINTS $RUN/map.pcd
wc -l < $RUN/trajectory.tum
tail -1 $RUN/trajectory.tum
ros2 run map_builder voxel_count $RUN/map.pcd 0.05
```

What a correct run gives (measured on 2026-10-22):

| Check | Expected | Why it matters |
|---|---|---|
| `POINTS` in `map.pcd` | 45195 | one point per 5 cm cube of the walk |
| lines in `trajectory.tum` | 296 | one pose per scan Point-LIO processed |
| last line, 2nd column (x) | 5.30… | Point-LIO's end of the walk; the tape says 5.40 m |
| `voxel_count` | `Inserted 45195 points, voxels: 45195, second inserted: 0` | no two saved points share a cube, so the map has no repeats |
| last `seq` in terminal 2 | about 150 | depends on timing; only the gaps matter |

If `POINTS` is about 18,700 and x is about 7.36, Point-LIO ran with the old 0.5 filter size: check
`filter_size_surf` and `filter_size_map` in `go2_mapping_bringup/config/point_lio.yaml` (must be 0.1),
rebuild, and run again.

### 8. Optional: look at the map in RViz

In `go2_mapping_bringup/config/map_builder.yaml` set `full_map_rate_hz: 0.2`, rebuild the bringup
package, and start the test again. In a fourth terminal (same four lines as step 2):

```bash
QT_QPA_PLATFORM=xcb rviz2
```

Type `camera_init` as Fixed Frame, add a PointCloud2 display on `/map_builder/full_map`, and set its
Durability to *Transient Local* (so RViz gets the last map even if it starts late). Walls must be
thin and straight, not doubled. Set `full_map_rate_hz` back to 0 afterwards: every message holds
the whole map.

## When something goes wrong

| Symptom | Cause | Fix |
|---|---|---|
| `N publishers detected on topic /map_builder/delta: another map builder is running`, then the node stops | a map_builder from an earlier run is still alive | `pgrep -af map_builder`, stop it, start again |
| `save` answers `map is empty` | no scan arrived | is Point-LIO running? `ros2 topic hz /cloud_registered` while the bag plays |
| terminal 2 prints nothing | the bag is not playing, or the terminals use different `ROS_DOMAIN_ID` / RMW | check the exports in all terminals |
| `Frame ID mismatch: expected camera_init, got ...` | something else publishes on `/cloud_registered` | find it with `ros2 topic info -v /cloud_registered` |
| `ros2 node list` shows nodes that you stopped | the ROS 2 daemon remembers old nodes | `ros2 node list --no-daemon`, or `ros2 daemon stop` |
