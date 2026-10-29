# go2_calibration

Measures how the L1 lidar and its IMU sit on a Unitree Go2 and writes the result as a ROS 2
parameter file for `go2_preprocessor`. Run it once per robot, and again after a firmware update, a
remount of the lidar or a hard fall.

Nothing about the frames is written in the code: every rotation is measured from data and every
result is checked. That is the difference from CMU's `calibrate_imu`, which keeps its robot's lidar
rotation in the code and only measures bias and leak inside it; on our firmware that rotation is
wrong by about 25 deg and CMU's method cannot see it.

## What it measures

| Result | From | Check (default limit) |
|---|---|---|
| `lidar_to_body` (rotation, translation) | `/utlidar/cloud` against its copy `/utlidar/cloud_base`, robot still | matched points (at least 1000), fit error rms (at most 1 mm) |
| floor tilt | the floor in the raw cloud, turned by `lidar_to_body` | at most 2 deg |
| `range_offset` | how the flat floor bends into a bowl (every range a constant too long) | reported, no limit |
| `body_imu.gyro_bias` | body gyroscope of `/sportmodestate`, robot still | noise std (at most 0.02 rad/s) |
| `body_imu.accel_scale` | 9.81 over the length of the still body accelerometer reading | at most 0.1 away from 1 |
| body gravity direction | still body accelerometer against +z | at most 5 deg |
| `imu_to_body` (rotation, report only) | lidar IMU gyroscope against the body gyroscope, robot turning and tilting, with a lag search over ±100 ms | fit error rms (at most 0.25 rad/s), lag inside the search (at most 95 ms), turn axis spread (at least 0.01) |
| `gyro_bias` (report only) | lidar IMU gyroscope, robot still, in the body frame | noise std (at most 0.02 rad/s) |
| `gyro_leak` (report only) | what `imu_to_body` and the bias leave of the yaw rate on x and y | at most 0.1 |
| lidar gravity direction (report only) | lidar IMU accelerometer after `imu_to_body` | none: this accelerometer is synthetic |

`go2_preprocessor` feeds Point-LIO the body IMU only. The lidar IMU results stay as comments in the
file: the angle between `imu_to_body` and `lidar_to_body` shows whether the body IMU axes agree with
the lidar mount, because the lidar IMU uses the cloud's axes (0.9 deg apart on our Go2).

If a check fails, the report shows FAIL and **no file is written**.

## How it works

```
             live: drives the robot          bag: replays a recorded bag
                 run_live()                        read_bag()
                      \                              /
                       set_phase / add_lidar_imu / add_body_imu / add_scan
                                     |
                              CalibrationBuffer   (sorts samples by phase: still, moving, none)
                                     |
                               calibrate()  ->  checks  ->  go2_calibration.yaml
```

`main` is one linear flow: read the settings, fill the buffer from one of the two sources,
calibrate, print the report, write the file if every check passed. The two sources make the same
calls on the buffer, so the calibration never knows where its data came from.

The live run drives the robot with the sport API through these phases (defaults, about 97 s):

| Phase | Command | Default | Used for |
|---|---|---|---|
| `still` | StopMove | 30 s | bias, lidar mount, floor, range offset |
| `spin_left`, `spin_right` | Move(0, 0, +-1 rad/s) | 2 turns each (12.6 s) | IMU rotation, leak |
| `pause` | StopMove | 3 s | (dropped) |
| `balance_stand` | BalanceStand, sent once | 1 s | (dropped) |
| `tilt_pitch`, `tilt_roll` | Euler, 0.2 rad sine at 0.4 Hz | 15 s each | IMU rotation about the other two axes |
| `level` | Euler(0, 0, 0) | 2 s | (dropped) |

It publishes the phase name on `/calibration/phase` at 50 Hz, so a bag recorded alongside can be
replayed later with `bag:=`.

## Run

`tools/mapping/calibration.sh` does all of this section in one command: it records the bag, runs
the calibration and points `~/calibration/current.yaml` at the result for `map.sh`
([tools/mapping](../../tools/mapping/README.md)). By hand:

On the robot, standing on a flat floor with 1 m clear around it:

```bash
source ~/unitree_ros2/setup.sh
source ~/unitree_go2/install/setup.bash
ros2 run go2_calibration go2_calibration
```

It warns, counts down 5 s (Ctrl-C cancels, nothing moves), drives the robot, prints the report and
writes `go2_calibration.yaml` in the current folder. To keep the raw data as well, record alongside
in a second terminal and stop it after the run:

```bash
ros2 bag record -o ~/bags/$(date +%F)_calibration_01 \
    /utlidar/cloud /utlidar/cloud_base /utlidar/imu /sportmodestate /calibration/phase
```

From such a bag, on the robot or the laptop, without moving anything:

```bash
ros2 run go2_calibration go2_calibration --ros-args -p bag:=<bag folder> -p output:=<file>
```

Load the result in `go2_preprocessor` with `--ros-args --params-file go2_calibration.yaml` (or
`parameters=[...]` in a launch file).

## Parameters

All are ROS parameters (`-p name:=value`, or `--params-file`); the defaults suit any Go2 with the L1.

| Parameter | Default | Meaning |
|---|---|---|
| `bag` | empty | empty: live run; a bag folder: calibrate from it |
| `output` | `go2_calibration.yaml` | the file written |
| `node_name` | `go2_preprocessor` | the node the parameter file is for |
| `max_scans` | 100 | matched scans kept for the lidar mount |
| `topics.cloud`, `topics.cloud_base`, `topics.imu` | `/utlidar/...` | lidar cloud, its body-frame copy, lidar IMU |
| `topics.body_state`, `topics.request`, `topics.phase` | `/sportmodestate`, `/api/sport/request`, `/calibration/phase` | body IMU, sport commands, phase name |
| `motion.still_s`, `motion.pause_s` | 30, 3 | s (10 to 300, 1 to 30) |
| `motion.spin_rate`, `motion.spin_turns` | 1.0, 2 | rad/s (0.3 to 1.5), turns each way (0.5 to 5) |
| `motion.tilt_angle`, `motion.tilt_frequency`, `motion.tilt_s` | 0.2, 0.4, 15 | rad (0 to 0.3; 0 leaves the tilts out), Hz (0.1 to 1), s (2 to 60) |
| `limits.*` | see the table above | `min_matched_points`, `max_mount_error_mm`, `max_floor_tilt_deg`, `max_gyro_error`, `min_axis_spread`, `max_still_noise`, `max_leak`, `max_accel_scale_error`, `max_body_tilt_deg` |

A motion value outside its range stops the program before anything moves.

## Output

```yaml
# Go2 calibration for go2_preprocessor, written by go2_calibration on <date>
# from <bag or "a live run">.
# ... the report and the checks, as comments ...
go2_preprocessor:
  ros__parameters:
    lidar_to_body:
      rotation: [9 values, row-major]
      translation: [x, y, z]
    body_imu:
      gyro_bias: {x, y, z}
      accel_scale: <about 1.04>
    range_offset: <metres>
```

Convention: `p_body = R * p_sensor + t`, body frame x forward, y left, z up. `body_imu` needs no
rotation: the `/sportmodestate` IMU already uses the body axes. `accel_scale` is a scale and not a
bias because standing still shows only the reading along gravity, where the two look the same.
`go2_preprocessor` moves every lidar point that far back towards the lidar, before `lidar_to_body`;
a point closer than the offset is dropped.

## Safety (live run)

- 5 s countdown before anything moves; Ctrl-C cancels.
- It waits up to 5 s for every sensor topic and moves nothing if one is silent.
- Every way out after that sends StopMove for 0.5 s: the end, Ctrl-C or SIGTERM, `/sportmodestate`
  silent for 0.5 s, and errors. `main` handles Ctrl-C itself, because rclcpp's own handler would shut
  ROS down before the robot got StopMove.
- Speeds and angles have hard limits (1.5 rad/s, 0.3 rad, 1 Hz) that no parameter can exceed.
- Not tried on the robot yet (2026-10-13): keep the remote in hand on the first run. Unitree ties
  Euler to the balance-stand mode; if the body does not tilt, the turns still give a result.

## Build

The package needs Unitree's `unitree_api` and `unitree_go` messages. On the robot they come with
`~/unitree_ros2`:

```bash
cd ~/unitree_go2
source ~/unitree_ros2/setup.sh
colcon build --packages-select go2_calibration
```

Laptop (foxy container), once: `sudo apt install ros-foxy-rosidl-generator-dds-idl`, clone
`https://github.com/unitreerobotics/unitree_ros2` and build only the messages
(`cd unitree_ros2/cyclonedds_ws && colcon build --packages-select unitree_api unitree_go`), then
source `cyclonedds_ws/install/local_setup.bash` after `/opt/ros/foxy/setup.bash`. The public
`SportModeState` lacks the `PathPoint[10]` tail of the robot's, which does not matter: the gyroscope
comes before it.

## Tests

```bash
colcon build --packages-select go2_calibration
colcon test --packages-select go2_calibration && colcon test-result --verbose
```

72 unit tests: every ROS-free part (sequence, buffer, fits, range offset, checks, report), the
message conversions, and the bag source on bags written by the test. The live source drives a robot
and has no unit test; it was checked on the laptop against fake topics on an isolated domain
(`ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77`): no robot, full run, and Ctrl-C mid-turn.

## Results on our Go2 (2026-10-13)

The 2026-10-13 `calib`, `standstill` and `spin` bags, merged into one bag with phase messages,
give in bag mode (all checks PASS):

- `lidar_to_body` = R_y(165 deg) R_z(-123 deg), t = (0.282, 0, 0), error rms 0.000 mm;
  floor tilt 0.82 deg.
- `imu_to_body` 0.65 deg from `lidar_to_body` (the IMU gyroscope uses the lidar axes), error rms
  0.123 rad/s, axis spread 0.040 (from the turns' leg wobble, before the tilts existed).
- `gyro_bias` (0.0028, -0.0055, 0.0006) rad/s, noise up to 0.007 rad/s; leak 0.034 / 0.009.
- Gravity 154 deg from +z after `imu_to_body`: the lidar accelerometer is not in the gyroscope
  frame, which is why `go2_preprocessor` takes the body IMU instead.
- `range_offset` 0.110 m with the lidar 0.330 m above the floor.

The body IMU, measured on the three 2026-10-19 calibration bags: `accel_scale` 1.0385 to 1.0420
(the still accelerometer reads 9.41 to 9.45 m/s^2), gravity 1.2 to 2.2 deg from +z, gyro bias about
(-0.010, 0.011, -0.004) rad/s. Only `calibration_03` passes every check; the other two fail the
floor tilt (2.05 and 2.83 deg), and `calibration_02` also the still noise.

## Limits

- `range_offset` assumes one constant offset for every range. Over all floor points it gives 0.110 m;
  over the floor within 1.6 m only, an earlier analysis gave 0.128 m, so the offset may change with
  range. Check it against a wall at several taped distances before relying on it.
- The bag source needs `/calibration/phase`; bags recorded without it (the old calib, standstill and
  spin bags) must be merged with phase messages first.
- The lidar mount comes from the firmware's own `/utlidar/cloud_base`, so it is the firmware's value
  rather than an independent measurement; the floor tilt is the independent check.

## Code

| File | Does | ROS |
|---|---|---|
| `sequence` | the phases, the command at each moment, which phases are still or moving | no |
| `buffer` | `CalibrationBuffer`: samples sorted by phase, scans matched with their copy | no |
| `fitting` | Kabsch rotation fit (vectors) and rigid fit (points) | no |
| `floor` | RANSAC floor plane with a least-squares refit, tilt from +z, range offset | no |
| `imu_fit` | pairing by time, IMU rotation with lag search, axis spread, statistics, leak | no |
| `calibrate` | the whole calibration and its checks | no |
| `report` | the report lines and the parameter file | no |
| `messages` | topic names, message conversions, sport requests | yes |
| `bag_source` | `read_bag()`: a bag into the buffer | yes |
| `live_source` | `run_live()`: drives the robot and fills the buffer | yes |
| `main` | settings, the linear flow, Ctrl-C handling | yes |

The ROS-free files form `go2_calibration_core`, the others `go2_calibration_ros`; the executable and
the tests link them.
