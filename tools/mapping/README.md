# Mapping session scripts

The two commands of a mapping session on the robot. Both run on the robot computer, from the
repository clone, inside tmux so a Wi-Fi drop does not kill them:

```bash
ssh go2
tmux new -s map
~/unitree_go2/tools/mapping/calibration.sh     # 1. the robot moves by itself for about 100 s
~/unitree_go2/tools/mapping/map.sh             # 2. walk; Ctrl-C stops and saves the map
```

| Script | Does | Writes |
|---|---|---|
| `calibration.sh` | runs [go2_calibration](../../src/go2_calibration/README.md) live, records its raw data | `~/calibration/<date>_calibration_NN.yaml`, the link `~/calibration/current.yaml`, the bag `~/bags/<date>_calibration_NN` |
| `map.sh [--bag]` | runs `mapping.launch.py` (preprocessor, Point-LIO, map_builder) with `current.yaml` until Ctrl-C | `~/maps/<time of the first scan, UTC>/map.pcd` and `trajectory.tum`; with `--bag` also `~/bags/<date>_map_NN` |
| `common.sh` | (sourced) the shared checks | |

The two run at different moments and share one file: `current.yaml`. Calibrate when the lidar was
moved, and at the start of a session until it is known how much the body gyroscope bias changes
between days. `map.sh` prints the age of the calibration it uses.

## Before the first session

The workspace must be built on the robot, with Point-LIO:

```bash
cd ~/unitree_go2
source ~/unitree_ros2/setup.sh
vcs import src < go2.repos
colcon build --packages-up-to go2_mapping_bringup go2_calibration
```

The scripts source `~/unitree_ros2/setup.sh` and `install/setup.bash` themselves.

## calibration.sh

1. Checks the lidar and body topics, that no calibration or mapping runs already, the clock (the
   date names the files) and the free disk.
2. Waits for Enter, then records the bag and runs the calibration. Ctrl-C at any moment stops the
   robot.
3. Every check passes: the new file is written and `current.yaml` points to it. A check fails, or
   Ctrl-C: no file, `current.yaml` keeps pointing to the old one, and the bag is kept to run the
   calibration again offline (`-p bag:=<bag folder>` on the laptop).
4. Prints the commands to copy the file and the bag to the laptop.

## map.sh

1. Checks that `current.yaml` exists, that nothing runs already, the topics, the `/utlidar/cloud`
   rate (about 15.4 Hz) and the free disk.
2. Waits for Enter with the robot standing still, starts the stack, and prints KEEP THE ROBOT STILL
   for the first 10 s: launch takes about 3 s, then Point-LIO measures gravity and builds its first
   map from the next seconds.
3. Prints GO. Walk. To stop: keep still 2 s, then press Ctrl-C once and wait for the summary.
4. map_builder saves the map on the way out. The script prints the folder, the number of points,
   how far from the start the path ends (the drift, for a walk that ends on its start mark) and the
   command to copy the folder to the laptop.

`--bag` also records `/utlidar/cloud` and `/sportmodestate` (about 110 MiB a minute), so the same
walk can be replayed on the laptop with other settings.

Point-LIO ends with `terminate called without an active exception` and `process has died`. That is
a known flaw of its exit, after the map is saved.

## Overrides

| Variable | Default |
|---|---|
| `ROS_SETUP` | `~/unitree_ros2/setup.sh` |
| `WS_SETUP` | `install/setup.bash` at the repository root |
| `CALIBRATION_DIR` | `~/calibration` |
| `BAG_DIR` | `~/bags` |
| `MAP_DIR` | `~/maps` |

## Tested

On the laptop, 2026-10-28, in the `foxy` container with `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77`,
bags played in place of the robot:

- `calibration.sh` with `2026-10-19_calibration_03`: every check passed, `current.yaml` moved to the
  new file, the bag has `metadata.yaml`. Stopped with Ctrl-C after 20 s: no file, `current.yaml`
  unchanged, the bag kept.
- `map.sh --bag` with `2026-10-19_line_01` at `-r 0.25`: Ctrl-C saved 44729 points and 297 poses,
  path end 5.34 m from the start (tape 5.40 m), the bag has `metadata.yaml`.

Not yet run on the robot.
